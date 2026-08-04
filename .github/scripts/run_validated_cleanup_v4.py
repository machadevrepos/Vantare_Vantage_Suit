#!/usr/bin/env python3
from __future__ import annotations

import base64
import gzip
import hashlib
from pathlib import Path
import shutil
import subprocess

import apply_validated_firmware_cleanup as cleanup

ALPHABET = b'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/='


def blob_sha(data: bytes) -> str:
    return hashlib.sha1(f'blob {len(data)}\0'.encode('ascii') + data).hexdigest()


def exact_blob(blob: str) -> bytes | None:
    try:
        return subprocess.check_output(
            ['git', 'cat-file', 'blob', blob], cwd=cleanup.REPO)
    except subprocess.CalledProcessError:
        return None


def repair_fragment(name: str, source: bytes, target: str) -> bytes:
    if blob_sha(source) == target:
        return source

    stored = exact_blob(target)
    if stored is not None and blob_sha(stored) == target:
        print(f'{name}: recovered exact blob from Git object database', flush=True)
        return stored

    print(
        f'{name}: repairing current blob {blob_sha(source)} to recorded blob {target}',
        flush=True,
    )

    # Uploaded transport fragments have only suffered single-character damage.
    # Each candidate is accepted only when its full Git blob SHA-1 is exact.
    for position in range(len(source) + 1):
        prefix = source[:position]
        suffix = source[position:]
        for value in ALPHABET:
            candidate = prefix + bytes((value,)) + suffix
            if blob_sha(candidate) == target:
                print(
                    f'{name}: inserted {chr(value)!r} at byte {position}',
                    flush=True,
                )
                return candidate

    for position in range(len(source)):
        candidate = source[:position] + source[position + 1:]
        if blob_sha(candidate) == target:
            print(f'{name}: deleted byte {position}', flush=True)
            return candidate

    for position, existing in enumerate(source):
        prefix = source[:position]
        suffix = source[position + 1:]
        for value in ALPHABET:
            if value == existing:
                continue
            candidate = prefix + bytes((value,)) + suffix
            if blob_sha(candidate) == target:
                print(
                    f'{name}: replaced byte {position} with {chr(value)!r}',
                    flush=True,
                )
                return candidate

    raise RuntimeError(
        f'{name}: unable to reconstruct recorded blob {target} from a single-character repair'
    )


def reconstruct_cleanup_patch() -> Path:
    pieces: dict[str, bytes] = {}
    for name in cleanup.PARTS:
        current = subprocess.check_output(
            ['git', 'show', f'{cleanup.APPLY_REF}:.cleanup-patch/{name}'],
            cwd=cleanup.REPO,
        )
        target = cleanup.PART_BLOBS[name]
        repaired = repair_fragment(name, current, target)
        if blob_sha(repaired) != target:
            raise RuntimeError(f'{name}: repaired blob verification failed')
        pieces[name] = repaired

    encoded = b''.join(pieces[name] for name in cleanup.PARTS)
    patch = gzip.decompress(base64.b64decode(encoded, validate=False))
    actual_sha = hashlib.sha256(patch).hexdigest()
    if actual_sha != cleanup.PATCH_SHA256:
        raise RuntimeError(
            f'cleanup patch SHA-256 mismatch: expected {cleanup.PATCH_SHA256}, got {actual_sha}'
        )
    patch_path = Path('/tmp/ble-cleanup-validated.patch')
    patch_path.write_bytes(patch)
    print(f'cleanup patch SHA-256 verified: {actual_sha}', flush=True)
    return patch_path


original_remove = cleanup.remove_transport_machinery


def remove_transport_machinery() -> None:
    original_remove()
    for relative in [
        '.github/workflows/execute-validated-cleanup-v2.yml',
        '.github/workflows/execute-validated-cleanup-v4.yml',
        '.github/workflows/actions-probe.yml',
        '.github/scripts/materialize_validated_firmware_cleanup.py',
        '.github/scripts/prepare_executor_v2.py',
        '.github/scripts/run_validated_cleanup_v4.py',
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
