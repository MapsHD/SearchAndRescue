#!/usr/bin/env python3

import os
import subprocess
import sys

from repo_files import CLANG_FORMAT_EXECUTABLE, PROJECT_DIR, ClangFormatError, find_pinned_clang_format, get_accepted_files

# Warning name clang-format attaches to every formatting violation it reports
CLANG_FORMAT_VIOLATION_MARKER = "-Wclang-format-violations"

EXIT_CODE_FORMATTED = 0
EXIT_CODE_NOT_FORMATTED = 1
EXIT_CODE_ERROR = 2


def run(command: list[str]):
    return subprocess.run(
        command,
        shell=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding="utf-8",
        errors="replace",
    )


def main():
    try:
        clang_format, version = find_pinned_clang_format()
    except ClangFormatError as error:
        print(f"ERROR: {error}")
        return EXIT_CODE_ERROR

    print(f"Using {CLANG_FORMAT_EXECUTABLE} {version}: {clang_format}")

    files = get_accepted_files()

    print(f"Checking {len(files)} files...")

    unformatted_files = []

    for file_path in files:
        # --dry-run leaves the file untouched; --Werror makes any needed change a non-zero exit
        result = run([clang_format, "--dry-run", "--Werror", file_path])

        if result.returncode == 0:
            continue

        relative_path = os.path.relpath(file_path, PROJECT_DIR)

        # Anything other than a formatting violation (bad .clang-format, unreadable file, unknown option)
        # means the check itself cannot be trusted
        if CLANG_FORMAT_VIOLATION_MARKER not in result.stderr:
            print(f"ERROR: {CLANG_FORMAT_EXECUTABLE} failed on {relative_path}")
            print(result.stderr)
            return EXIT_CODE_ERROR

        unformatted_files.append(relative_path)

    if unformatted_files:
        print(f"ERROR: {CLANG_FORMAT_EXECUTABLE} would change the following files:")

        for file_path in unformatted_files:
            print(f"  {file_path}")

        print("Run scripts/run_clang_format.py to format them")
        return EXIT_CODE_NOT_FORMATTED

    print("clang-format check passed")
    return EXIT_CODE_FORMATTED


if __name__ == "__main__":
    sys.exit(main())
