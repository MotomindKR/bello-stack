from __future__ import annotations

from collections.abc import Sequence

import grpc  # type: ignore[import-untyped]
from bello_hardware_api import hardware_pb2 as proto


class BelloError(Exception):
    """Base class for all package errors."""


class ConnectionError(BelloError):
    """The daemon could not be reached."""


class RequestError(BelloError):
    """A request to the hardware daemon failed."""

    def __init__(self, operation: str, code: str, detail: str) -> None:
        self.operation = operation
        self.code = code
        self.detail = detail
        super().__init__(f"{operation} failed ({code}): {detail}")


class ProtocolError(BelloError):
    """The daemon returned a response that violates the protocol contract."""


class InventoryError(BelloError):
    """The hardware inventory is inconsistent."""


class InvalidTargetError(BelloError, ValueError):
    """A coordinate target or complete group frame is invalid."""


class ControlError(BelloError):
    """A motor-group control operation failed."""


class ControlRequestError(RequestError, ControlError):
    """A request for motor-group control failed."""


class InvalidSessionStateError(ControlError):
    """An operation is not valid in the session's current state."""


class LeaseLostError(ControlError):
    """Exclusive control authority expired or was rejected."""


class CommandQueueFullError(ControlError):
    """All pending-command slots are occupied."""


class AcknowledgementError(ControlError):
    """A command acknowledgement was missing, late, or inconsistent."""


class CommandRejectedError(ControlError):
    """The daemon explicitly rejected a complete command frame."""

    def __init__(
        self,
        *,
        group: str,
        sequence: int,
        result: proto.CommandResult,
        detail: str,
        state: proto.GroupState,
    ) -> None:
        self.group = group
        self.sequence = sequence
        self.result = result
        self.detail = detail
        self.state = state
        result_name = proto.CommandResult.Name(result)
        state_name = proto.GroupState.Name(state)
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


def rpc_error(operation: str, error: grpc.RpcError) -> RequestError:
    code = error.code().name
    detail = error.details() or str(error)
    return RequestError(operation, code, detail)


def control_rpc_error(operation: str, error: grpc.RpcError) -> ControlError:
    if error.code() == grpc.StatusCode.PERMISSION_DENIED:
        return LeaseLostError(f"{operation} lost its motor-group lease: {error.details()}")
    return ControlRequestError(operation, error.code().name, error.details() or str(error))
