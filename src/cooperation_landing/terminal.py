"""Terminal input shared by interactive ROS nodes."""

import select
import sys
import termios
import tty


def get_key(settings, timeout=0.1):
    """Read a key, or return an empty string so the caller can check shutdown.

    Restore the terminal even when polling or reading raises an exception.
    ``settings`` must come from ``termios.tcgetattr(sys.stdin)``.
    """
    try:
        tty.setraw(sys.stdin.fileno())
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if readable else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
