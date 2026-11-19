"""Choose a HUD endpoint without waiting for an earlier HTTP session."""
import errno
import socket


def choose_hud_port(preferred):
    # UE HTTPServer may not reuse TIME_WAIT sockets. Probe without SO_REUSEADDR
    # so the chosen port is usable even when the server doesn't enable it.
    with socket.socket() as probe:
        try:
            probe.bind(('0.0.0.0', preferred))
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
            probe.bind(('0.0.0.0', 0))
        return probe.getsockname()[1]
