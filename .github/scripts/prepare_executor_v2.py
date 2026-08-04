#!/usr/bin/env python3
from pathlib import Path

path = Path('.github/scripts/apply_validated_firmware_cleanup.py')
text = path.read_text()

old = '''    for name in PARTS:
        raw = subprocess.check_output(
            ['git', 'show', f'{APPLY_REF}:.cleanup-patch/{name}'],
            cwd=REPO,
        )
        wanted = PART_BLOBS[name]
        actual = git_blob_sha(raw)
        if name != 'part-06b.b64' and actual != wanted:
            raise RuntimeError(f'{name}: expected blob {wanted}, got {actual}')
        if name == 'part-06b.b64' and actual != wanted:
'''
new = '''    for name in PARTS:
        wanted = PART_BLOBS[name]
        if name == 'part-06b.b64':
            raw = subprocess.check_output(
                ['git', 'show', f'{APPLY_REF}:.cleanup-patch/{name}'],
                cwd=REPO,
            )
        else:
            raw = subprocess.check_output(
                ['git', 'cat-file', 'blob', wanted],
                cwd=REPO,
            )
        actual = git_blob_sha(raw)
        if name != 'part-06b.b64' and actual != wanted:
            raise RuntimeError(f'{name}: expected blob {wanted}, got {actual}')
        if name == 'part-06b.b64' and actual != wanted:
'''
if text.count(old) != 1:
    raise SystemExit('cleanup reconstruction block missing')
text = text.replace(old, new, 1)

anchor = "        '.github/workflows/run-validated-firmware-cleanup.yml',\n"
additions = (
    "        '.github/workflows/execute-validated-cleanup-v2.yml',\n"
    "        '.github/workflows/execute-validated-cleanup-v3.yml',\n"
    "        '.github/workflows/actions-probe.yml',\n"
    "        '.github/scripts/materialize_validated_firmware_cleanup.py',\n"
    "        '.github/scripts/prepare_executor_v2.py',\n"
    "        'actions-probe.txt',\n"
    "        'cleanup-executor.log',\n"
)
if text.count(anchor) != 1:
    raise SystemExit('cleanup transport-removal anchor missing')
path.write_text(text.replace(anchor, anchor + additions, 1))
