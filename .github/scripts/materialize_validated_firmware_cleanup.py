#!/usr/bin/env python3
from __future__ import annotations

import shutil
from pathlib import Path

import apply_validated_firmware_cleanup as cleanup


def materialize(_: str) -> str:
    repository = cleanup.REPO
    worktree = cleanup.WORK

    for child in list(repository.iterdir()):
        if child.name == '.git':
            continue
        if child.is_dir() and not child.is_symlink():
            shutil.rmtree(child)
        else:
            child.unlink()

    for child in worktree.iterdir():
        if child.name == '.git':
            continue
        destination = repository / child.name
        if child.is_dir() and not child.is_symlink():
            shutil.copytree(child, destination, symlinks=True)
        else:
            shutil.copy2(child, destination, follow_symlinks=False)

    return 'materialized-validated-tree'


cleanup.commit_and_push = materialize
raise SystemExit(cleanup.main())
