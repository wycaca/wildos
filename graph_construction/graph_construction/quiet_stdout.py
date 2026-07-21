from __future__ import annotations

import os
import sys


def main() -> None:
    """Replace the current process while discarding noisy standard output"""
    if len(sys.argv) < 2:
        raise RuntimeError("quiet_stdout requires a command")
    command = sys.argv[1]
    arguments = sys.argv[1:]
    with open(os.devnull, "wb", buffering=0) as sink:
        os.dup2(sink.fileno(), sys.stdout.fileno())
        os.execvp(command, arguments)


if __name__ == "__main__":
    main()
