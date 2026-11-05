#!/usr/bin/env python3

import subprocess
import sys

from repo_files import CLANG_FORMAT_EXECUTABLE, ClangFormatError, find_pinned_clang_format, list_files


def format_file(clang_format: str, file_path: str):
    print(f" - formatting: {file_path}")

    command = [clang_format, "-i", file_path]
    return subprocess.run(command, shell=False).returncode


def format_files(clang_format: str, file_paths: list[str]):
    failed_files = []

    for file_path in file_paths:
        if format_file(clang_format, file_path) != 0:
            failed_files.append(file_path)

    return failed_files


def main():
    try:
        clang_format, version = find_pinned_clang_format()
    except ClangFormatError as error:
        print(f"ERROR: {error}")
        return 1

    print(f"Using {CLANG_FORMAT_EXECUTABLE} {version}: {clang_format}")
    print("Searching files...")

    files, excluded_files = list_files()

    for file_path in excluded_files:
        print(f" - skipping: {file_path}")

    print(f"Found {len(files)} files to format")

    failed_files = format_files(clang_format, files)

    if failed_files:
        print(f"ERROR: {CLANG_FORMAT_EXECUTABLE} failed on {len(failed_files)} file(s):")

        for file_path in failed_files:
            print(f"  {file_path}")

        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
