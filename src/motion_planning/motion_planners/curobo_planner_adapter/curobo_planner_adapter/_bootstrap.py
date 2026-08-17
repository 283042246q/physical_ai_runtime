"""Path bootstrap for uv-managed cuRobo/Torch/Warp dependencies."""

from __future__ import annotations

import glob
import sys
from pathlib import Path


def _find_workspace_root() -> Path:
    path = Path(__file__).resolve()
    for parent in path.parents:
        if (parent / "pixi.toml").exists():
            return parent
    raise RuntimeError(
        "Could not locate workspace root (no pixi.toml found above "
        f"{Path(__file__).resolve()})"
    )


def ensure_curobo_importable() -> None:
    workspace_root = _find_workspace_root()
    venv_dir = workspace_root / ".venv"
    if not venv_dir.exists():
        return
    pattern = str(venv_dir / "lib" / "python*" / "site-packages")
    for site_packages_dir in glob.glob(pattern):
        if site_packages_dir not in sys.path:
            sys.path.insert(0, site_packages_dir)


ensure_curobo_importable()
