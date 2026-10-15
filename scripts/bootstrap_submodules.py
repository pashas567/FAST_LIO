#!/usr/bin/env python3
"""Restore pinned ikd-Tree from a reviewed local delta bundle."""

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHILD = ROOT / "include/ikd-Tree"
TARGET = "e72230bdca1845371441927ded16e406a25ceffd"
BUNDLE = ROOT / "dependencies/ikd_tree_startup.bundle"


def git(directory: Path, *arguments: str) -> str:
    return subprocess.check_output(
        [
            "git",
            "-c",
            "core.hooksPath=/dev/null",
            "-C",
            str(directory),
            *arguments,
        ],
        text=True,
    ).strip()


def main() -> None:
    git(ROOT, "submodule", "init")
    created = not (CHILD / ".git").exists()
    if created:
        url = git(
            ROOT,
            "config",
            "--file",
            ".gitmodules",
            "--get",
            "submodule.include/ikd-Tree.url",
        )
        subprocess.run(
            ["git", "clone", "--no-checkout", "--", url, str(CHILD)],
            check=True,
        )
    if not created and git(CHILD, "status", "--porcelain"):
        raise SystemExit(
            f"{CHILD}: preserve local changes before updating the submodule"
        )
    # The prerequisite is upstream e2e3f4e. Only reviewed source deltas
    # are included; the bundle contains no build artifacts.
    git(CHILD, "bundle", "verify", str(BUNDLE))
    git(
        CHILD,
        "fetch",
        "--no-recurse-submodules",
        "--no-tags",
        str(BUNDLE),
        "refs/remotes/transfer/avia-ground-start",
    )
    git(CHILD, "checkout", "--detach", TARGET)
    git(ROOT, "submodule", "absorbgitdirs", "include/ikd-Tree")
    print(f"ikd-Tree ready: {TARGET}")


if __name__ == "__main__":
    main()
