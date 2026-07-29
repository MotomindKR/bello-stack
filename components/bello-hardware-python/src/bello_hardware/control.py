"""Blocking, automatically renewed motor-group control sessions."""

from __future__ import annotations

import math
import queue
import threading
import time
import warnings
from collections import deque
from collections.abc import Iterator, Mapping
from contextlib import suppress
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal

import grpc  # type: ignore[import-untyped]

from ._generated import hardware_pb2 as proto
from .errors import (
    AcknowledgementError,
    CleanupError,
    CommandRejectedError,
    ControlError,
    InvalidSessionStateError,
    LeaseLostError,
    ProtocolError,
    control_rpc_error,
)
from .types import CommandResult, ControlState, CoordinateTarget, GroupState

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


@dataclass(slots=True)
class _Pending:
    event: threading.Event
    submitted_at: float
    completed_at: float | None = None
    response: Any = None
    error: BaseException | None = None


@dataclass(slots=True)
class CommandReceipt:
    """Acknowledgement handle for a command submitted without blocking."""

    _session: Any
    _sequence: int
    _pending: _Pending
    _deadline: float
    _resolved: bool = False

    def done(self) -> bool:
        return self._pending.event.is_set()

    @property
    def acknowledgement_latency_ms(self) -> float | None:
        """Return submit-to-acknowledgement latency, or None while pending."""

        completed_at = self._pending.completed_at
        if completed_at is None:
            return None
        return max(0.0, completed_at - self._pending.submitted_at) * 1_000.0

    def result(self) -> None:
        if self._resolved:
            return
        self._session._resolve_receipt(self)
        self._resolved = True


class _CommandStream:
    _SENTINEL = object()

    def __init__(self, stub: Any, group: str, token: str) -> None:
        self._group = group
        self._requests: queue.Queue[object] = queue.Queue(maxsize=1)
        self._pending: dict[int, _Pending] = {}
        self._lock = threading.Lock()
        self._fatal: BaseException | None = None
        self._closing = False
        self._call = stub.StreamCommands(self._request_iterator())
        self._receiver = threading.Thread(
            target=self._receive,
            name=f"bello-command-{group}",
            daemon=True,
        )
        self._receiver.start()

    def _request_iterator(self) -> Iterator[proto.StreamCommandsRequest]:
        while True:
            request = self._requests.get()
            if request is self._SENTINEL:
                return
            if not isinstance(request, proto.StreamCommandsRequest):
                raise ProtocolError("invalid value in the command request queue")
            yield request

    def submit(self, request: proto.StreamCommandsRequest, timeout: float) -> _Pending:
        pending = _Pending(threading.Event(), submitted_at=time.monotonic())
        with self._lock:
            if self._fatal is not None:
                raise self._fatal
            if self._closing:
                raise AcknowledgementError("command stream is closed")
            self._pending[request.sequence] = pending
        try:
            self._requests.put(request, timeout=timeout)
        except queue.Full as error:
            with self._lock:
                self._pending.pop(request.sequence, None)
            raise AcknowledgementError("timed out submitting the command") from error

        return pending

    def wait(self, sequence: int, pending: _Pending, timeout: float) -> Any:
        if timeout <= 0.0 or not pending.event.wait(timeout):
            with self._lock:
                self._pending.pop(sequence, None)
            raise AcknowledgementError(f"timed out waiting for command {sequence} acknowledgement")
        if pending.error is not None:
            raise pending.error
        return pending.response

    def send(self, request: proto.StreamCommandsRequest, timeout: float) -> Any:
        pending = self.submit(request, timeout)
        return self.wait(request.sequence, pending, timeout)

    def _receive(self) -> None:
        try:
            for response in self._call:
                if response.group != self._group:
                    self._fail(
                        ProtocolError(
                            f"command response group {response.group!r} does not match "
                            f"{self._group!r}"
                        )
                    )
                    return
                with self._lock:
                    pending = self._pending.pop(response.sequence, None)
                if pending is None:
                    self._fail(
                        AcknowledgementError(
                            f"received unexpected command acknowledgement {response.sequence}"
                        )
                    )
                    return
                pending.response = response
                pending.completed_at = time.monotonic()
                pending.event.set()
            if not self._closing:
                self._fail(AcknowledgementError("command acknowledgement stream closed"))
        except grpc.RpcError as error:
            if not self._closing:
                self._fail(control_rpc_error("command stream", error))
        except BaseException as error:
            if not self._closing:
                self._fail(error)

    def _fail(self, error: BaseException) -> None:
        with self._lock:
            if self._fatal is None:
                self._fatal = error
            pending = tuple(self._pending.values())
            self._pending.clear()
        for item in pending:
            item.error = error
            item.completed_at = time.monotonic()
            item.event.set()

    def close(self, timeout: float) -> None:
        with self._lock:
            if self._closing:
                return
            self._closing = True
            pending = tuple(self._pending.values())
            self._pending.clear()
        error = AcknowledgementError("command stream closed")
        for item in pending:
            item.error = error
            item.completed_at = time.monotonic()
            item.event.set()
        with suppress(queue.Full):
            self._requests.put_nowait(self._SENTINEL)
        self._call.cancel()
        if self._receiver is not threading.current_thread():
            self._receiver.join(timeout=timeout)


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
        acknowledgement_timeout: float,
        rpc_timeout: float,
        request_started: float,
    ) -> None:
        self._group = group
        self._token = token
        self._lease_duration_ms = lease_duration_ms
        self._command_validity_ms = command_validity_ms
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
        self._next_sequence = 1
        self._renewal = threading.Thread(
            target=self._renewal_loop,
            name=f"bello-lease-{group.name}",
            daemon=True,
        )
        self._renewal.start()

    @classmethod
    def acquire(
        cls,
        group: MotorGroup,
        owner: str,
        *,
        lease_duration: float | None,
        command_validity: float | None,
        acknowledgement_timeout: float,
        rpc_timeout: float,
    ) -> ControlSession:
        if not owner.strip():
            raise ValueError("owner must be non-empty")
        if acknowledgement_timeout <= 0.0 or rpc_timeout <= 0.0:
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
            self._expect("enable", GroupState.DAMPED, response.state)
            self._set_state(ControlState.DAMPED)
            try:
                self._stream = _CommandStream(
                    self._group._robot._new_stub(), self._group.name, self._token
                )
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
                self._expect("start", GroupState.RUNNING, response.state)
                self._set_state(ControlState.RUNNING)
            except BaseException as primary:
                self._close_stream()
                try:
                    response = self._unary(
                        "rollback start",
                        self._group._robot._stub.DisableGroup,
                        proto.DisableGroupRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("rollback start", GroupState.IDLE, response.state)
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
            self._stream = _CommandStream(
                self._group._robot._new_stub(), self._group.name, self._token
            )
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
                self._expect("resume", GroupState.RUNNING, response.state)
                self._set_state(ControlState.RUNNING)
            except BaseException as primary:
                self._close_stream()
                try:
                    response = self._unary(
                        "rollback resume",
                        self._group._robot._stub.StopControl,
                        proto.StopControlRequest(group=self._group.name, lease_token=self._token),
                    )
                    self._expect("rollback resume", GroupState.DAMPED, response.state)
                    self._set_state(ControlState.DAMPED)
                except BaseException as cleanup:
                    self._set_state(ControlState.UNCERTAIN)
                    raise CleanupError((primary, cleanup)) from primary
                raise

    def send(self, targets: Mapping[str, CoordinateTarget]) -> None:
        commands = self._group._commands(targets)
        with self._operation_lock:
            self._require(ControlState.RUNNING, "send a command")
            try:
                self._check_renewal()
                self._send_and_confirm(commands)
            except BaseException as error:
                self._close_stream()
                self._set_state(self._state_after_command_error(error))
                raise

    def send_async(self, targets: Mapping[str, CoordinateTarget]) -> CommandReceipt:
        """Submit a complete target batch and return immediately with its receipt."""
        commands = self._group._commands(targets)
        with self._operation_lock:
            self._require(ControlState.RUNNING, "send a command")
            self._check_renewal()
            stream = self._stream
            if stream is None:
                raise self._invalid("send a command")
            sequence = self._next_sequence
            if sequence > 0xFFFF_FFFF_FFFF_FFFF:
                raise ControlError("command sequence is exhausted")
            self._next_sequence += 1
            request = proto.StreamCommandsRequest(
                group=self._group.name,
                lease_token=self._token,
                sequence=sequence,
                valid_for_ms=self._command_validity_ms,
                commands=commands,
            )
            pending = stream.submit(request, self._acknowledgement_timeout)
            return CommandReceipt(
                self,
                sequence,
                pending,
                time.monotonic() + self._acknowledgement_timeout,
            )

    def stop(self) -> None:
        with self._operation_lock:
            self._require(ControlState.RUNNING, "stop")
            self._close_stream()
            response = self._unary(
                "stop control",
                self._group._robot._stub.StopControl,
                proto.StopControlRequest(group=self._group.name, lease_token=self._token),
            )
            self._expect("stop", GroupState.DAMPED, response.state)
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
            self._expect("disable", GroupState.IDLE, response.state)
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
            self._expect("recover", GroupState.IDLE, response.state)
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
                    self._expect("stop", GroupState.DAMPED, response.state)
                except BaseException as error:
                    errors.append(error)
            if not preserve_damping:
                try:
                    response = self._unary(
                        "disable group",
                        self._group._robot._stub.DisableGroup,
                        proto.DisableGroupRequest(
                            group=self._group.name, lease_token=self._token
                        ),
                    )
                    self._expect("disable", GroupState.IDLE, response.state)
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
                    (GroupState.IDLE, GroupState.DAMPED, GroupState.FAULT)
                    if preserve_damping
                    else (GroupState.IDLE,)
                )
                self._expect_any("release", expected, response.state)
            except BaseException as error:
                errors.append(error)
            self._set_state(ControlState.CLOSED)
            self._group._robot._unregister(self)
            if errors:
                raise CleanupError(errors)

    shutdown = close

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

    def _send_and_confirm(self, commands: list[proto.MotorCommand]) -> None:
        self._check_renewal()
        stream = self._stream
        if stream is None:
            raise self._invalid("send a command")
        sequence = self._next_sequence
        if sequence > 0xFFFF_FFFF_FFFF_FFFF:
            raise ControlError("command sequence is exhausted")
        self._next_sequence += 1
        response = stream.send(
            proto.StreamCommandsRequest(
                group=self._group.name,
                lease_token=self._token,
                sequence=sequence,
                valid_for_ms=self._command_validity_ms,
                commands=commands,
            ),
            self._acknowledgement_timeout,
        )
        self._validate_response(sequence, response)
        self._check_renewal()

    def _resolve_receipt(self, receipt: CommandReceipt) -> None:
        stream = self._stream
        if stream is None:
            raise self._invalid("await a command acknowledgement")
        try:
            response = stream.wait(
                receipt._sequence,
                receipt._pending,
                receipt._deadline - time.monotonic(),
            )
            self._validate_response(receipt._sequence, response)
            self._check_renewal()
        except BaseException as error:
            self._close_stream()
            self._set_state(self._state_after_command_error(error))
            raise

    def _validate_response(self, sequence: int, response: Any) -> None:
        if response.sequence != sequence:
            raise AcknowledgementError(
                f"received acknowledgement {response.sequence}, expected {sequence}"
            )
        try:
            result = CommandResult(response.result)
            state = GroupState(response.state)
        except ValueError as error:
            raise ProtocolError("command acknowledgement contains an unknown enum value") from error
        if result != CommandResult.ACCEPTED:
            raise CommandRejectedError(
                group=self._group.name,
                sequence=sequence,
                result=result,
                detail=response.detail,
                state=state,
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
                    response = self._group._robot._new_stub().RenewLease(
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

    def _expect(self, operation: str, expected: GroupState, actual_value: int) -> None:
        self._expect_any(operation, (expected,), actual_value)

    def _expect_any(
        self,
        operation: str,
        expected: tuple[GroupState, ...],
        actual_value: int,
    ) -> None:
        try:
            actual = GroupState(actual_value)
        except ValueError as error:
            raise ProtocolError(
                f"{operation} returned unknown group state {actual_value}"
            ) from error
        if actual not in expected:
            names = ", ".join(state.name for state in expected)
            raise ControlError(
                f"{operation} for {self._group.name!r} returned {actual.name}, "
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

    @staticmethod
    def _state_after_command_error(error: BaseException) -> ControlState:
        if isinstance(error, CommandRejectedError):
            if error.state == GroupState.RUNNING:
                return ControlState.RUNNING
            if error.state == GroupState.DAMPED:
                return ControlState.DAMPED
        return ControlState.UNCERTAIN

    def _close_stream(self) -> None:
        stream, self._stream = self._stream, None
        if stream is not None:
            stream.close(self._rpc_timeout)


class CommandPipeline:
    """Bounded non-blocking command submission for fixed-rate control loops."""

    def __init__(self, session: ControlSession, *, max_pending: int) -> None:
        if isinstance(max_pending, bool) or not isinstance(max_pending, int):
            raise TypeError("maximum pending commands must be an integer")
        if max_pending <= 0:
            raise ValueError("maximum pending commands must be positive")
        self._session = session
        self._max_pending = max_pending
        self._pending: deque[CommandReceipt] = deque()
        self._acknowledgement_latencies_ms: deque[float] = deque(maxlen=256)
        self._last_submission_latency_ms = 0.0
        self._maximum_submission_latency_ms = 0.0

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def acknowledgement_latencies_ms(self) -> tuple[float, ...]:
        return tuple(self._acknowledgement_latencies_ms)

    @property
    def last_submission_latency_ms(self) -> float:
        return self._last_submission_latency_ms

    @property
    def maximum_submission_latency_ms(self) -> float:
        return self._maximum_submission_latency_ms

    def send(self, targets: Mapping[str, CoordinateTarget]) -> CommandReceipt:
        """Submit one frame without waiting unless the pending bound is full."""

        self.poll()
        started_at = time.monotonic()
        receipt = self._session.send_async(targets)
        submission_latency_ms = (time.monotonic() - started_at) * 1_000.0
        self._last_submission_latency_ms = submission_latency_ms
        self._maximum_submission_latency_ms = max(
            self._maximum_submission_latency_ms,
            submission_latency_ms,
        )
        self._pending.append(receipt)
        while len(self._pending) > self._max_pending:
            self._resolve(self._pending.popleft())
        return receipt

    def poll(self) -> None:
        """Resolve completed receipts and surface command failures."""

        while self._pending and self._pending[0].done():
            self._resolve(self._pending.popleft())

    def drain(self) -> None:
        """Wait for and validate every pending acknowledgement."""

        while self._pending:
            self._resolve(self._pending.popleft())

    def _resolve(self, receipt: CommandReceipt) -> None:
        receipt.result()
        latency_ms = receipt.acknowledgement_latency_ms
        if latency_ms is not None:
            self._acknowledgement_latencies_ms.append(latency_ms)
