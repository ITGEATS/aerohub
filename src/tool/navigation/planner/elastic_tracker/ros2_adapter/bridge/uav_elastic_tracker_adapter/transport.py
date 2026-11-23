"""Same loopback newline JSON RPC transport as CAT-ORA / Swarm proxies."""
import json
import socket
from .timing import trace


def rpc(host, port, payload):
    # Existing Swarm bridge socket deadline; this is not a sensor age policy.
    with socket.create_connection((host, port), timeout=0.5) as connection:
        traced = payload.get('kind') == 'latest'
        port_id = connection.getsockname()[1]
        if traced:
            trace(None, 'tcp_connected', port=port_id)
        stream = connection.makefile('rwb')
        stream.write((json.dumps(payload, allow_nan=False, separators=(',', ':')) + '\n').encode())
        stream.flush()
        if traced:
            trace(None, 'tcp_request_sent', port=port_id)
        response = json.loads(stream.readline())
        if traced:
            trace(None, 'tcp_response_decoded', port=port_id,
                  source_id=(response.get('trajectory') or {}).get('traj_id'))
    if not response['ok']:
        raise RuntimeError(response.get('error', 'proxy error'))
    return response
