#!/usr/bin/env python3
from __future__ import annotations

import base64
import gzip
import hashlib
from pathlib import Path
import shutil
import subprocess

import apply_validated_firmware_cleanup as cleanup


def read_fragment(name: str) -> bytes:
    data = subprocess.check_output(
        ['git', 'show', f'{cleanup.APPLY_REF}:.cleanup-patch/{name}'],
        cwd=cleanup.REPO,
    )
    print(f'{name}: bytes={len(data)}', flush=True)
    return data


def decode_candidate(label: str, names: list[str], fragments: dict[str, bytes]) -> bytes | None:
    encoded = b''.join(fragments[name] for name in names)
    try:
        compressed = base64.b64decode(encoded, validate=False)
        patch = gzip.decompress(compressed)
    except Exception as exc:
        print(f'{label}: decode failed: {exc}', flush=True)
        return None
    actual = hashlib.sha256(patch).hexdigest()
    print(f'{label}: decompressed_bytes={len(patch)} sha256={actual}', flush=True)
    return patch if actual == cleanup.PATCH_SHA256 else None


def reconstruct_cleanup_patch() -> Path:
    layouts: list[tuple[str, list[str]]] = [
        (
            'combined-06',
            [
                'part-01.b64', 'part-02.b64', 'part-03.b64', 'part-04.b64',
                'part-05.b64', 'part-06.b64',
                'part-07.b64', 'part-08.b64', 'part-09.b64',
            ],
        ),
        (
            'split-06a-06b1-06b2',
            [
                'part-01.b64', 'part-02.b64', 'part-03.b64', 'part-04.b64',
                'part-05.b64', 'part-06a.b64', 'part-06b1.b64', 'part-06b2.b64',
                'part-07.b64', 'part-08.b64', 'part-09.b64',
            ],
        ),
        (
            'split-06a-06b',
            [
                'part-01.b64', 'part-02.b64', 'part-03.b64', 'part-04.b64',
                'part-05.b64', 'part-06a.b64', 'part-06b.b64',
                'part-07.b64', 'part-08.b64', 'part-09.b64',
            ],
        ),
    ]

    fragments: dict[str, bytes] = {}
    for _, names in layouts:
        for name in names:
            if name not in fragments:
                fragments[name] = read_fragment(name)

    patch: bytes | None = None
    selected = ''
    for label, names in layouts:
        patch = decode_candidate(label, names, fragments)
        if patch is not None:
            selected = label
            break
    if patch is None:
        raise RuntimeError(
            'none of the redundant uploaded layouts reconstructed the recorded patch SHA-256 '
            f'{cleanup.PATCH_SHA256}'
        )

    print(
        f'{selected}: authoritative cleanup patch SHA-256 verified: {cleanup.PATCH_SHA256}',
        flush=True,
    )
    patch_path = Path('/tmp/ble-cleanup-validated.patch')
    patch_path.write_bytes(patch)
    return patch_path


original_remove = cleanup.remove_transport_machinery


def remove_transport_machinery() -> None:
    original_remove()
    for relative in [
        '.github/workflows/execute-validated-cleanup-v2.yml',
        '.github/workflows/execute-validated-cleanup-v6.yml',
        '.github/workflows/actions-probe.yml',
        '.github/scripts/materialize_validated_firmware_cleanup.py',
        '.github/scripts/prepare_executor_v2.py',
        '.github/scripts/run_validated_cleanup_v4.py',
        '.github/scripts/run_validated_cleanup_v5.py',
        '.github/scripts/run_validated_cleanup_v6.py',
        'actions-probe.txt',
        'cleanup-executor.log',
    ]:
        target = cleanup.WORK / relative
        if target.is_dir() and not target.is_symlink():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()
    for relative in ['.github/scripts', '.github/workflows', '.github']:
        directory = cleanup.WORK / relative
        if directory.exists() and directory.is_dir() and not any(directory.iterdir()):
            directory.rmdir()


cleanup.reconstruct_cleanup_patch = reconstruct_cleanup_patch
cleanup.remove_transport_machinery = remove_transport_machinery
raise SystemExit(cleanup.main())
