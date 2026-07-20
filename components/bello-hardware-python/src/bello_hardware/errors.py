"""Public exception hierarchy for the Bello client."""

from __future__ import annotations

from collections.abc import Sequence

import grpc  # type: ignore[import-untyped]


class BelloError(Exception):
    """Base class for all package errors."""


class ConnectionError(BelloError):
    """The daemon could not be reached."""


class RpcError(BelloError):
    """A gRPC operation failed."""

    def __init__(self, operation: str, code: grpc.StatusCode, detail: str) -> None:
        self.operation = operation
        self.code = code
        self.detail = detail
        super().__init__(f"{operation} failed ({code.name}): {detail}")


class ProtocolError(BelloError):
    """The daemon returned a response that violates the protocol contract."""


class InventoryError(BelloError):
    """The hardware inventory is inconsistent."""


class InvalidTargetError(BelloError, ValueError):
    """A coordinate target or complete group frame is invalid."""


class ControlError(BelloError):
    """A motor-group control operation failed."""


class ControlRpcError(ControlError):
    """A motor-group gRPC operation failed."""

    def __init__(self, operation: str, code: grpc.StatusCode, detail: str) -> None:
        self.operation = operation
        self.code = code
        self.detail = detail
        super().__init__(f"{operation} failed ({code.name}): {detail}")


class InvalidSessionStateError(ControlError):
    """An operation is not valid in the session's current state."""


class LeaseLostError(ControlError):
    """Exclusive control authority expired or was rejected."""


class AcknowledgementError(ControlError):
    """A command acknowledgement was missing, late, or inconsistent."""


class CommandRejectedError(ControlError):
    """The daemon explicitly rejected a complete command frame."""

    def __init__(
        self,
        *,
        group: str,
        sequence: int,
        result: object,
        detail: str,
        state: object,
    ) -> None:
        self.group = group
        self.sequence = sequence
        self.result = result
        self.detail = detail
        self.state = state
        result_name = getattr(result, "name", str(result))
        state_name = getattr(state, "name", str(state))
        super().__init__(
            f"command {sequence} for {group!r} was rejected as {result_name} "
            f"in {state_name}: {detail}"
        )


class CleanupError(ControlError):
    """One or more best-effort control cleanup operations failed."""

    def __init__(self, errors: Sequence[BaseException | str]) -> None:
        self.errors = tuple(errors)
        super().__init__("control cleanup failed: " + "; ".join(map(str, self.errors)))


class RobotCleanupError(BelloError):
    """One or more robot-owned resources failed to close."""

    def __init__(self, errors: Sequence[BaseException | str]) -> None:
        self.errors = tuple(errors)
        super().__init__("robot cleanup failed: " + "; ".join(map(str, self.errors)))


class StateUnavailableError(BelloError):
    """No current telemetry is available for a configured component."""


class WorkerShutdownError(BelloError):
    """A background client worker did not stop before its deadline."""


def rpc_error(operation: str, error: grpc.RpcError) -> RpcError:
    code = error.code()
    detail = error.details() or str(error)
    return RpcError(operation, code, detail)


def control_rpc_error(operation: str, error: grpc.RpcError) -> ControlError:
    if error.code() == grpc.StatusCode.PERMISSION_DENIED:
        return LeaseLostError(f"{operation} lost its motor-group lease: {error.details()}")
    return ControlRpcError(operation, error.code(), error.details() or str(error))
