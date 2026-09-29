"""Apply literal text substitutions to source files.

The editor tooling in this environment cannot reliably rewrite files in this
checkout (ReplaceFileW returns Win32 error 1175), so patches are applied with
plain Python IO instead. Every substitution is asserted to match exactly once
so a silent no-op is impossible.

Line endings are normalised to ``\\n`` before matching and writing, so a patch
descriptor can be written with plain newlines regardless of what the target
file uses.

Usage:  python tools/patch.py <patchfile.json>

The patch file is a JSON list of objects:
    {"file": "safe_serial.py", "old": "...", "new": "...", "count": 1}
Optionally "dry_run": true on an entry to only report what it would do.
"""

import io
import json
import os
import sys


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2

    with io.open(argv[1], encoding='utf-8') as handle:
        patches = json.load(handle)

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    failures = []

    for index, patch in enumerate(patches):
        path = os.path.join(root, patch['file'])
        with io.open(path, encoding='utf-8', newline='') as handle:
            raw = handle.read()
        text = raw.replace('\r\n', '\n')

        old = patch['old'].replace('\r\n', '\n')
        new = patch['new'].replace('\r\n', '\n')
        expected = patch.get('count', 1)
        found = text.count(old)

        if found != expected:
            failures.append('%s[%d]: expected %d match(es), found %d'
                            % (patch['file'], index, expected, found))
            continue

        if patch.get('dry_run'):
            print('would patch %s[%d]' % (patch['file'], index))
            continue

        with io.open(path, 'w', encoding='utf-8', newline='\n') as handle:
            handle.write(text.replace(old, new))
        print('patched %s[%d]' % (patch['file'], index))

    if failures:
        print('FAILED:')
        for failure in failures:
            print('  ' + failure)
        return 1

    print('all %d patch(es) applied' % len(patches))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
