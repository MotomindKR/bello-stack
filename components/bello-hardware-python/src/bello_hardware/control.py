"""Blocking, automatically renewed motor-group control sessions."""

from __future__ import annotations

import math
import threading
import time
import warnings
from collections import deque
from collections.abc import Iterator, Mapping
from contextlib import suppress
from itertools import count
from typing import TYPE_CHECKING, Any, Literal

import grpc  # type: ignore[import-untyped]

from ._generated import hardware_pb2 as proto
from .errors import (
    AcknowledgementError,
    CleanupError,
    CommandQueueFullError,
    CommandRejectedError,
    ControlError,
    InvalidSessionStateError,
    LeaseLostError,
    ProtocolError,
    control_rpc_error,
)
from .types import ControlState, CoordinateTarget

if TYPE_CHECKING:
    from ._inventory import MotorGroup


def _milliseconds(seconds: float | None, field: str) -> int:
    if seconds is None:
        return 0
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError(f"{field} must be positive and finite")
    milliseconds = int(seconds * 1_000.0)
    if milliseconds == 0:
        raise ValueError(f"{field} must be at least one millisecond")
    return milliseconds


class CommandReceipt:
    """The daemon's acknowledgement for one submitted command."""

    def __init__(self) -> None:
        self._submitted = time.monotonic()
        self._event = threading.Event()
        self._completed: float | None = None
        self._error: BaseException | None = None

    def done(self) -> bool:
        return self._event.is_set()

    def result(self) -> None:
        self._event.wait()
        if self._error is not None:
            raise self._error

    @property
    def acknowledgement_latency_ms(self) -> float | None:
        if self._completed is None:
            return None
        return (self._completed - self._submitted) * 1000

    def _finish(self, error: BaseException | None, acknowledged_at: float | None = None) -> None:
        self._error = error
        self._completed = acknowledged_at
        self._event.set()


class _CommandStream:
    def __init__(self, session: ControlSession) -> None:
        self._session = session
        self._changed = threading.Condition()
        self._requests: deque[proto.StreamCommandsRequest] = deque()
        self._pending: dict[int, CommandReceipt] = {}
        self._closed: BaseException | None = None
        self._call = session.group._robot._stub.StreamCommands(self._request_iterator())
        self._receiver = threading.Thread(target=self._receive, daemon=True)
        self._watchdog = threading.Thread(target=self._watch_deadlines, daemon=True)
        self._receiver.start()
        self._watchdog.start()

    def _request_iterator(self) -> Iterator[proto.StreamCommandsRequest]:
        while True:
            with self._changed:
                self._changed.wait_for(lambda: self._closed is not None or bool(self._requests))
                if self._closed is not None:
                    return
                request = self._requests.popleft()
            yield request

    def mark_running(self) -> None:
        with self._changed:
            if self._closed is not None:
                raise self._closed
            self._session._check_renewal()
            self._session._set_state(ControlState.RUNNING)

    def submit(self, commands: list[proto.MotorCommand], *, wait: bool) -> CommandReceipt:
        session = self._session
        with self._changed:
            while self._closed is None and len(self._pending) >= session._max_pending:
                if not wait:
                    raise CommandQueueFullError("maximum pending command count reached")
                self._changed.wait()
            if self._closed is not None:
                raise self._closed
            try:
                session._check_renewal()
            except LeaseLostError as error:
                self._fail(error)
                raise
            sequence = next(session._sequences)
            request = proto.StreamCommandsRequest(
                group=session.group.name,
                lease_token=session._token,
                sequence=sequence,
                valid_for_ms=session._command_validity_ms,
                commands=commands,
            )
            receipt = CommandReceipt()
            self._pending[sequence] = receipt
            self._requests.append(request)
            self._changed.notify_all()
            return receipt

    def _watch_deadlines(self) -> None:
        with self._changed:
            while self._closed is None:
                first = next(iter(self._pending.items()), None)
                if first is None:
                    self._changed.wait()
                    continue
                sequence, receipt = first
                remaining = (
                    receipt._submitted + self._session._acknowledgement_timeout - time.monotonic()
                )
                if remaining <= 0:
                    self._fail(self._timeout(sequence))
                    return
                self._changed.wait(remaining)

    @staticmethod
    def _timeout(sequence: int) -> AcknowledgementError:
        return AcknowledgementError(f"timed out waiting for command {sequence} acknowledgement")

    def _receive(self) -> None:
        try:
            for response in self._call:
                with self._changed:
                    if self._closed is not None:
                        return
                    now = time.monotonic()
                    receipt = self._pending.get(response.sequence)
                    if response.group != self._session.group.name or receipt is None:
                        raise AcknowledgementError(
                            f"unexpected acknowledgement {response.group!r}/{response.sequence}"
                        )
                    if now > receipt._submitted + self._session._acknowledgement_timeout:
                        raise self._timeout(response.sequence)
                    self._session._check_renewal()
                    try:
                        self._session._validate_response(response)
                    except CommandRejectedError as error:
                        self._fail(error, response.sequence)
                        return
                    receipt._finish(None, now)
                    del self._pending[response.sequence]
                    self._changed.notify_all()
            self._fail(AcknowledgementError("command acknowledgement stream closed"))
        except grpc.RpcError as error:
            self._fail(control_rpc_error("command stream", error))
        except BaseException as error:
            self._fail(error)

    def _fail(self, error: BaseException, rejected_sequence: int | None = None) -> None:
        with self._changed:
            if self._closed is not None:
                return
            self._session._stream_failed(self, error)
            self._finish(error, rejected_sequence)
        self._call.cancel()

    def _finish(self, error: BaseException, rejected_sequence: int | None = None) -> None:
        self._closed = error
        for sequence, receipt in self._pending.items():
            if rejected_sequence is None or rejected_sequence == sequence:
                receipt._finish(error)
            else:
                receipt._finish(AcknowledgementError("command stream closed after rejection"))
        self._pending.clear()
        self._requests.clear()
        self._changed.notify_all()

    def close(self) -> None:
        with self._changed:
            if self._closed is None:
                self._finish(AcknowledgementError("command stream closed"))
        self._call.cancel()
        self._receiver.join(timeout=self._session._rpc_timeout)
        self._watchdog.join(timeout=self._session._rpc_timeout)


class ControlSession:
    """Exclusive blocking control authority for one motor group."""

    def __init__(
        self,
        group: MotorGroup,
        *,
        token: str,
        lease_duration_ms: int,
        expires_in_ms: int,
        command_validity_ms: int,
        max_pending: int,
        acknowledgement_timeout: float,
        rpc_timeout: float,
        request_started: float,
    ) -> None:
        self._group = group
        self._token = token
        self._lease_duration_ms = lease_duration_ms
        self._command_validity_ms = command_validity_ms
        self._max_pending = max_pending
        self._acknowledgement_timeout = acknowledgement_timeout
        self._rpc_timeout = rpc_timeout
        self._operation_lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._release_lock = threading.Lock()
        self._state = ControlState.READY
        self._deadline = request_started + expires_in_ms / 1_000.0
        self._renewal_error: BaseException | None = None
        self._renewal_stop = threading.Event()
        self._stream: _CommandStream | None = None
        self._sequences = count(1)
        self._renewal = threading.Thread(
            target=self._renewal_loop,
            name=f"bello-lease-{group.name}",
            daemon=True,
        )
        self._renewal.start()

    @classmethod
    def _acquire(
        cls,
        group: MotorGroup,
        owner: str,
        *,
        lease_duration: float | None,
        command_validity: float | None,
        max_pending: int,
        acknowledgement_timeout: float,
        rpc_timeout: float,
    ) -> ControlSession:
        if isinstance(max_pending, bool) or not isinstance(max_pending, int):
            raise TypeError("maximum pending commands must be an integer")
        if max_pending <= 0:
            raise ValueError("maximum pending commands must be positive")
        if not (0 < acknowledgement_timeout < math.inf and 0 < rpc_timeout < math.inf):
            raise ValueError("acknowledgement and RPC timeouts must be positive")
        lease_duration_ms = _milliseconds(lease_duration, "lease duration")
        command_validity_ms = _milliseconds(command_validity, "command validity")
        robot = group._robot
        robot._begin_child()
        admitted = False
        response: Any = None
        started = time.monotonic()
        try:
            response = robot._stub.AcquireLease(
                proto.AcquireLeaseRequest(
                    group=group.name,
                    client_name=owner,
                    requested_duration_ms=lease_duration_ms,
                ),
                timeout=rpc_timeout,
            )
            if response.group != group.name or not response.token or response.expires_in_ms == 0:
                raise ProtocolError("invalid acquire-lease response")
            session = cls(
                group,
                token=response.token,
                lease_duration_ms=lease_duration_ms,
                expires_in_ms=response.expires_in_ms,
                command_validity_ms=command_validity_ms,
                max_pending=max_pending,
                acknowledgement_timeout=acknowledgement_timeout,
                rpc_timeout=rpc_timeout,
                request_started=started,
            )
            robot._finish_child(session)
            admitted = True
            return session
        except grpc.RpcError as error:
            raise control_rpc_error("acquire lease", error) from error
        except BaseException:
            if response is not None and response.token:
                with suppress(grpc.RpcError):
                    robot._stub.ReleaseLease(
                        proto.ReleaseLeaseRequest(token=response.token),
                        timeout=rpc_timeout,
                    )
            raise
        finally:
            if not admitted:
                robot._finish_child(None)

    @property
    def group(self) -> MotorGroup:
        return self._group

    @property
    def state(self) -> ControlState:
        with self._state_lock:
            return self._state

    def start(self, targets: Mapping[str, CoordinateTarget]) -> None:
        commands = self._group._commands(targets)
        with self._operation_lock:
            self._require(ControlState.READY, "start")
            self._check_renewal()
            response = self._unary(
                "enable group",
                self._group._robot._stub.EnableGroup,
                proto.EnableGroupRequest(group=self._group.name, lease_token=self._token),
            )
            self._expect("enable", proto.GROUP_STATE_DAMPED, response.state)
            self._set_state(ControlState.DAMPED)
            try:
                self._stream = _CommandStream(self)
                response = self._unary(
                    "start control",
                    self._group._robot._stub.StartControl,
                    proto.StartControlRequest(
                        group=self._group.name,
                        lease_token=self._token,
                        valid_for_ms=self._command_validity_ms,
                        commands=commands,
                    ),
                )
                self._expect("start", proto.GROUP_STATE_RUNNING, response.state)
                self._stream.mark_running()
            except BaseException as primary:
                self._close_stream()
                try:
                    response = self._unary(
                        "rollback start",
                        self._group._robot._stub.DisableGroup,
                        proto.DisableGroupRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("rollback start", proto.GROUP_STATE_IDLE, response.state)
                    self._set_state(ControlState.READY)
                except BaseException as cleanup:
                    self._set_state(ControlState.UNCERTAIN)
                    raise CleanupError((primary, cleanup)) from primary
                raise

    def resume(self, targets: Mapping[str, CoordinateTarget]) -> None:
        commands = self._group._commands(targets)
        with self._operation_lock:
            self._require(ControlState.DAMPED, "resume")
            self._check_renewal()
            self._stream = _CommandStream(self)
            try:
                response = self._unary(
                    "resume control",
                    self._group._robot._stub.StartControl,
                    proto.StartControlRequest(
                        group=self._group.name,
                        lease_token=self._token,
                        valid_for_ms=self._command_validity_ms,
                        commands=commands,
                    ),
                )
                self._expect("resume", proto.GROUP_STATE_RUNNING, response.state)
                self._stream.mark_running()
            except BaseException as primary:
                self._close_stream()
                try:
                    response = self._unary(
                        "rollback resume",
                        self._group._robot._stub.StopControl,
                        proto.StopControlRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("rollback resume", proto.GROUP_STATE_DAMPED, response.state)
                    self._set_state(ControlState.DAMPED)
                except BaseException as cleanup:
                    self._set_state(ControlState.UNCERTAIN)
                    raise CleanupError((primary, cleanup)) from primary
                raise

    def send(self, targets: Mapping[str, CoordinateTarget]) -> None:
        """Wait for queue space and the daemon's acknowledgement."""
        self._submit(targets, wait=True).result()

    def try_submit(self, targets: Mapping[str, CoordinateTarget]) -> CommandReceipt:
        """Submit immediately or raise CommandQueueFullError when all slots are occupied."""
        return self._submit(targets, wait=False)

    def stop(self) -> None:
        with self._operation_lock:
            self._require(ControlState.RUNNING, "stop")
            self._close_stream()
            response = self._unary(
                "stop control",
                self._group._robot._stub.StopControl,
                proto.StopControlRequest(group=self._group.name, lease_token=self._token),
            )
            self._expect("stop", proto.GROUP_STATE_DAMPED, response.state)
            self._set_state(ControlState.DAMPED)

    def disable(self) -> None:
        with self._operation_lock:
            if self.state not in (ControlState.READY, ControlState.DAMPED):
                raise self._invalid("disable")
            self._close_stream()
            response = self._unary(
                "disable group",
                self._group._robot._stub.DisableGroup,
                proto.DisableGroupRequest(group=self._group.name, lease_token=self._token),
            )
            self._expect("disable", proto.GROUP_STATE_IDLE, response.state)
            self._set_state(ControlState.READY)

    def recover(self) -> None:
        with self._operation_lock:
            if self.state == ControlState.CLOSED:
                raise self._invalid("recover")
            self._close_stream()
            self._check_renewal()
            response = self._unary(
                "recover group",
                self._group._robot._stub.RecoverGroup,
                proto.RecoverGroupRequest(group=self._group.name, lease_token=self._token),
            )
            self._expect("recover", proto.GROUP_STATE_IDLE, response.state)
            self._set_state(ControlState.READY)

    def close(self, *, preserve_damping: bool = False) -> None:
        with self._operation_lock:
            if self.state == ControlState.CLOSED:
                return
            prior_state = self.state
            self._close_stream()
            errors: list[BaseException] = []
            if prior_state == ControlState.RUNNING:
                try:
                    response = self._unary(
                        "stop control",
                        self._group._robot._stub.StopControl,
                        proto.StopControlRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("stop", proto.GROUP_STATE_DAMPED, response.state)
                except BaseException as error:
                    errors.append(error)
            if not preserve_damping:
                try:
                    response = self._unary(
                        "disable group",
                        self._group._robot._stub.DisableGroup,
                        proto.DisableGroupRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("disable", proto.GROUP_STATE_IDLE, response.state)
                except BaseException as error:
                    errors.append(error)

            self._renewal_stop.set()
            if self._renewal is not threading.current_thread():
                self._renewal.join(timeout=self._rpc_timeout + 0.1)
            try:
                with self._release_lock:
                    response = self._unary(
                        "release lease",
                        self._group._robot._stub.ReleaseLease,
                        proto.ReleaseLeaseRequest(token=self._token),
                    )
                expected = (
                    (proto.GROUP_STATE_IDLE, proto.GROUP_STATE_DAMPED, proto.GROUP_STATE_FAULT)
                    if preserve_damping
                    else (proto.GROUP_STATE_IDLE,)
                )
                self._expect_any("release", expected, response.state)
            except BaseException as error:
                errors.append(error)
            self._set_state(ControlState.CLOSED)
            self._group._robot._unregister(self)
            if errors:
                raise CleanupError(errors)

    def __enter__(self) -> ControlSession:
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> Literal[False]:
        try:
            self.close()
        except CleanupError as cleanup:
            if exc is None:
                raise
            with suppress(Warning):
                warnings.warn(str(cleanup), RuntimeWarning, stacklevel=2)
        return False

    def _submit(self, targets: Mapping[str, CoordinateTarget], *, wait: bool) -> CommandReceipt:
        commands = self._group._commands(targets)
        with self._state_lock:
            if self._state != ControlState.RUNNING or self._stream is None:
                raise InvalidSessionStateError(f"cannot send a command while {self._state.value}")
            stream = self._stream
        return stream.submit(commands, wait=wait)

    def _validate_response(self, response: proto.StreamCommandsResponse) -> None:
        if response.state not in proto.GroupState.DESCRIPTOR.values_by_number:
            raise ProtocolError(f"unknown command acknowledgement state {response.state}")
        if response.result != proto.COMMAND_RESULT_ACCEPTED:
            if response.result not in proto.CommandResult.DESCRIPTOR.values_by_number:
                raise ProtocolError(f"unknown command result {response.result}")
            raise CommandRejectedError(
                group=self._group.name,
                sequence=response.sequence,
                result=response.result,
                detail=response.detail,
                state=response.state,
            )

    def _renewal_loop(self) -> None:
        while not self._renewal_stop.is_set():
            with self._state_lock:
                wait = max((self._deadline - time.monotonic()) / 2.0, 0.001)
            if self._renewal_stop.wait(wait):
                return
            started = time.monotonic()
            try:
                with self._release_lock:
                    response = self._group._robot._stub.RenewLease(
                        proto.RenewLeaseRequest(
                            token=self._token,
                            requested_duration_ms=self._lease_duration_ms,
                        ),
                        timeout=self._rpc_timeout,
                    )
                if (
                    response.group != self._group.name
                    or response.token != self._token
                    or response.expires_in_ms == 0
                ):
                    raise ProtocolError("invalid renew-lease response")
                with self._state_lock:
                    self._deadline = started + response.expires_in_ms / 1_000.0
            except grpc.RpcError as error:
                self._record_renewal_error(control_rpc_error("renew lease", error))
                return
            except BaseException as error:
                self._record_renewal_error(error)
                return

    def _record_renewal_error(self, error: BaseException) -> None:
        with self._state_lock:
            if self._renewal_error is None:
                self._renewal_error = error
            stream = self._stream
        if stream is not None:
            stream._fail(LeaseLostError(f"automatic lease renewal failed: {error}"))

    def _check_renewal(self) -> None:
        with self._state_lock:
            error = self._renewal_error
            expired = time.monotonic() >= self._deadline
        if error is not None:
            message = f"automatic lease renewal for {self._group.name!r} failed: {error}"
            raise LeaseLostError(message) from error
        if expired:
            raise LeaseLostError(f"lease for motor group {self._group.name!r} expired")

    def _unary(self, operation: str, method: Any, request: Any) -> Any:
        try:
            return method(request, timeout=self._rpc_timeout)
        except grpc.RpcError as error:
            raise control_rpc_error(operation, error) from error

    def _expect(self, operation: str, expected: proto.GroupState, actual_value: int) -> None:
        self._expect_any(operation, (expected,), actual_value)

    def _expect_any(
        self,
        operation: str,
        expected: tuple[proto.GroupState, ...],
        actual_value: int,
    ) -> None:
        if actual_value not in expected:
            names = ", ".join(proto.GroupState.Name(state) for state in expected)
            raise ControlError(
                f"{operation} for {self._group.name!r} returned state {actual_value}, "
                f"expected one of {names}"
            )

    def _require(self, expected: ControlState, operation: str) -> None:
        if self.state != expected:
            raise self._invalid(operation)

    def _invalid(self, operation: str) -> InvalidSessionStateError:
        return InvalidSessionStateError(
            f"cannot {operation} for {self._group.name!r} while session is {self.state.value}"
        )

    def _set_state(self, state: ControlState) -> None:
        with self._state_lock:
            self._state = state

    def _stream_failed(self, stream: _CommandStream, error: BaseException) -> None:
        with self._state_lock:
            if self._stream is stream:
                self._state = (
                    ControlState.DAMPED
                    if isinstance(error, CommandRejectedError)
                    and error.result != proto.COMMAND_RESULT_LEASE_LOST
                    and error.state == proto.GROUP_STATE_DAMPED
                    else ControlState.UNCERTAIN
                )

    def _close_stream(self) -> None:
        with self._state_lock:
            stream, self._stream = self._stream, None
        if stream is not None:
            stream.close()
