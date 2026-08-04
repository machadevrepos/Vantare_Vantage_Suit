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


def read_fragment(name: str) -> bytes:
    return subprocess.check_output(
        ['git', 'show', f'{cleanup.APPLY_REF}:.cleanup-patch/{name}'],
        cwd=cleanup.REPO,
    )


def repair_part_06b(source: bytes) -> bytes:
    target = cleanup.PART_BLOBS['part-06b.b64']
    if blob_sha(source) == target:
        return source
    for position in range(len(source) + 1):
        prefix = source[:position]
        suffix = source[position:]
        for value in ALPHABET:
            candidate = prefix + bytes((value,)) + suffix
            if blob_sha(candidate) == target:
                print(
                    f'part-06b.b64: inserted {chr(value)!r} at byte {position}; blob verified',
                    flush=True,
                )
                return candidate
    raise RuntimeError('part-06b.b64 could not be repaired to its recorded Git blob')


def decode_candidate(label: str, fragments: list[bytes]) -> bytes | None:
    encoded = b''.join(fragments)
    try:
        compressed = base64.b64decode(encoded, validate=False)
        patch = gzip.decompress(compressed)
    except Exception as exc:
        print(f'{label}: decode failed: {exc}', flush=True)
        return None
    actual = hashlib.sha256(patch).hexdigest()
    print(f'{label}: patch SHA-256 {actual}', flush=True)
    return patch if actual == cleanup.PATCH_SHA256 else None


def reconstruct_cleanup_patch() -> Path:
    current: dict[str, bytes] = {}
    for name in cleanup.PARTS:
        current[name] = read_fragment(name)
        expected = cleanup.PART_BLOBS[name]
        actual = blob_sha(current[name])
        if actual != expected:
            print(
                f'{name}: transport blob differs from recorded value '
                f'(recorded={expected}, current={actual}); full patch hash remains mandatory',
                flush=True,
            )

    current['part-06b.b64'] = repair_part_06b(current['part-06b.b64'])

    candidates: list[tuple[str, list[str]]] = [
        (
            'split-06a-06b',
            [
                'part-01.b64', 'part-02.b64', 'part-03.b64', 'part-04.b64',
                'part-05.b64', 'part-06a.b64', 'part-06b.b64',
                'part-07.b64', 'part-08.b64', 'part-09.b64',
            ],
        ),
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
    ]

    for _, names in candidates:
        for name in names:
            if name not in current:
                current[name] = read_fragment(name)

    patch: bytes | None = None
    for label, names in candidates:
        patch = decode_candidate(label, [current[name] for name in names])
        if patch is not None:
            print(
                f'{label}: authoritative cleanup patch SHA-256 verified: {cleanup.PATCH_SHA256}',
                flush=True,
            )
            break
    if patch is None:
        raise RuntimeError('no uploaded transport layout reconstructed the recorded cleanup patch SHA-256')

    patch_path = Path('/tmp/ble-cleanup-validated.patch')
    patch_path.write_bytes(patch)
    return patch_path


original_remove = cleanup.remove_transport_machinery


def remove_transport_machinery() -> None:
    original_remove()
    for relative in [
        '.github/workflows/execute-validated-cleanup-v2.yml',
        '.github/workflows/execute-validated-cleanup-v5.yml',
        '.github/workflows/actions-probe.yml',
        '.github/scripts/materialize_validated_firmware_cleanup.py',
        '.github/scripts/prepare_executor_v2.py',
        '.github/scripts/run_validated_cleanup_v4.py',
        '.github/scripts/run_validated_cleanup_v5.py',
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
