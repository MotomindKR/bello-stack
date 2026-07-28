import hashlib
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

_PACKAGE_ASSET_DIRECTORY = Path("src/bello_upper_body_teleop/_assets")
_URDF_NAME = "bello_full_body.urdf"
_PROVENANCE_NAME = "ASSET_PROVENANCE.md"
_SOURCE_URDF = Path("descriptions/bello/full_body/urdf") / _URDF_NAME
_COMMIT_PATTERN = re.compile(r"^- Stack source commit: `([0-9a-f]{40})`$", re.MULTILINE)
_DIGEST_PATTERN = re.compile(r"^- Embedded URDF SHA-256: `([0-9a-f]{64})`$", re.MULTILINE)


def _git_output(stack_root: Path, *arguments: str) -> bytes:
    try:
        return subprocess.run(
            ["git", "-C", stack_root, *arguments],
            check=True,
            capture_output=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError(f"failed to identify canonical Bello asset source: {error}") from error


def _checkout_assets(project_root: Path) -> tuple[Path, str]:
    stack_root = project_root.parents[1]
    repository_root = Path(
        _git_output(stack_root, "rev-parse", "--show-toplevel").decode().strip()
    ).resolve()
    if repository_root != stack_root.resolve():
        raise RuntimeError(f"teleop package is not inside the Bello stack checkout: {stack_root}")

    commit = _git_output(stack_root, "rev-parse", "--verify", "HEAD").decode().strip()
    if re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise RuntimeError(f"Git returned an invalid Bello stack commit: {commit!r}")

    urdf = stack_root / _SOURCE_URDF
    if not urdf.is_file():
        raise RuntimeError(f"canonical Bello URDF is unavailable: {urdf}")
    committed_urdf = _git_output(stack_root, "show", f"{commit}:{_SOURCE_URDF.as_posix()}")
    if urdf.read_bytes() != committed_urdf:
        raise RuntimeError(
            "canonical Bello URDF differs from the recorded source commit; commit it before "
            "building a teleop distribution"
        )
    return urdf, commit


def _render_provenance(commit: str, urdf: bytes) -> str:
    digest = hashlib.sha256(urdf).hexdigest()
    return f"""# Bello upper-body teleop asset provenance

This distribution contains only the canonical full-body URDF used to constrain
teleoperation joint positions. It does not contain the adjacent visual or
collision meshes referenced by that URDF.

- Source repository: `MotomindKR/bello-stack`
- Stack source commit: `{commit}`
- Source path: `{_SOURCE_URDF.as_posix()}`
- Embedded URDF SHA-256: `{digest}`
- License expression: `LicenseRef-Proprietary`

The embedded URDF is proprietary. This distribution includes the applicable
license notice; no additional permission to use or redistribute the asset is
granted.
"""


def _validate_bundled_assets(urdf: Path, provenance: Path) -> None:
    if not urdf.is_file() or not provenance.is_file():
        raise RuntimeError("teleop source distribution is missing its bundled asset snapshot")

    notice = provenance.read_text(encoding="utf-8")
    commit = _COMMIT_PATTERN.search(notice)
    digest = _DIGEST_PATTERN.search(notice)
    if commit is None or digest is None:
        raise RuntimeError("teleop source distribution has invalid asset provenance")
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != digest.group(1):
        raise RuntimeError("teleop source distribution URDF does not match its asset provenance")


class CanonicalAssetsBuildHook(BuildHookInterface):
    def initialize(self, version: str, build_data: dict[str, Any]) -> None:
        del version
        project_root = Path(self.root)
        bundled_directory = project_root / _PACKAGE_ASSET_DIRECTORY
        bundled_urdf = bundled_directory / _URDF_NAME
        bundled_provenance = bundled_directory / _PROVENANCE_NAME
        if (project_root / "PKG-INFO").is_file():
            _validate_bundled_assets(bundled_urdf, bundled_provenance)
            urdf = bundled_urdf
            provenance = bundled_provenance
        else:
            urdf, commit = _checkout_assets(project_root)
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                prefix="bello-upper-body-teleop-provenance-",
                suffix=".md",
                delete=False,
            ) as stream:
                stream.write(_render_provenance(commit, urdf.read_bytes()))
                provenance = Path(stream.name)
            self._generated_provenance = provenance

        destination_directory = (
            _PACKAGE_ASSET_DIRECTORY
            if self.target_name == "sdist"
            else Path("bello_upper_body_teleop/_assets")
        )
        for name, source in ((_URDF_NAME, urdf), (_PROVENANCE_NAME, provenance)):
            build_data["force_include"][str(source)] = (destination_directory / name).as_posix()

    def finalize(self, version: str, build_data: dict[str, Any], artifact_path: str) -> None:
        del version, build_data, artifact_path
        generated_provenance = getattr(self, "_generated_provenance", None)
        if generated_provenance is not None:
            generated_provenance.unlink(missing_ok=True)
