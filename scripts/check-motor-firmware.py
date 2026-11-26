"""Compare the embedded RobStride firmware with the latest stable vendor release."""

import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
catalog = json.loads((root / "components/bello-hardwared/firmware/catalog.json").read_text())
release = json.load(sys.stdin)
published = {
    asset["name"]: asset["digest"]
    for asset in release["assets"]
    if asset["name"].lower().startswith("rs") and asset["name"].lower().endswith(".bin")
}
bundled = {entry["filename"]: "sha256:" + entry["sha256"] for entry in catalog}
changes = [name for name, digest in published.items() if bundled.get(name) != digest]
if not published:
    sys.exit(f"No RS firmware assets recognized at {release['html_url']}. Check the release layout.")
if changes:
    sys.exit(f"Review firmware at {release['html_url']}: {', '.join(sorted(changes))}")
print(f"Embedded firmware matches the RS images published in {release['tag_name']}.")
