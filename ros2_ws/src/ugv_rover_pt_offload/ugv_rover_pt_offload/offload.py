# ----------------------------------------
# 1. Offload helper
# ----------------------------------------
# rsync folder to NAS over SSH, with retry and local marker files.

import time
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class OffloadTarget:
    host: str
    user: str
    dest_dir: str
    retries: int = 3
    backoff_sec: float = 2.0


def rsync_push_dir(local_dir: Path, target: OffloadTarget) -> str:
    remote_base = f"{target.user}@{target.host}:{target.dest_dir.rstrip('/')}/"
    cmd = [
        "rsync",
        "-avh",
        "--partial",
        "--append-verify",
        str(local_dir),
        remote_base,
    ]
    subprocess.check_call(cmd)
    return remote_base + local_dir.name


def offload_with_markers(local_dir: Path, target: OffloadTarget) -> str:
    uploading = local_dir.with_suffix(".uploading")
    complete = local_dir.with_suffix(".complete")

    uploading.write_text("uploading\n")

    last_err: Exception | None = None
    for attempt in range(1, target.retries + 1):
        try:
            remote_path = rsync_push_dir(local_dir, target)
            complete.write_text("complete\n")
            try:
                uploading.unlink(missing_ok=True)
            except Exception:
                pass
            return remote_path
        except Exception as e:
            last_err = e
            time.sleep(target.backoff_sec * attempt)

    raise RuntimeError(f"offload failed after {target.retries} attempts: {last_err}")
