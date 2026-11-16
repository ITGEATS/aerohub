"""SDK-free client for all Aerohub ProjectAirSim extension JSON-RPC methods."""
from http.client import HTTPException
from itertools import count
import json
from math import isfinite
from urllib.error import HTTPError, URLError
from urllib.request import Request, ProxyHandler, build_opener

GLOBAL_VEHICLE_ID = '__global__'


class ExtensionError(RuntimeError):
    """Base error for a ProjectAirSim extension request."""


class ExtensionTransportError(ExtensionError):
    """Connection, timeout or HTTP failure; delivery may have occurred."""


class ExtensionProtocolError(ExtensionError):
    """The endpoint returned an invalid JSON-RPC response."""


class ExtensionRPCError(ExtensionError):
    """The extension server rejected a request."""

    def __init__(self, code, message, data=None):
        self.code, self.message, self.data = code, message, data
        super().__init__(f'{code}: {message}')


class ProjectAirSimExtensionClient:
    """Synchronous extension RPC; no SDK, topic connection or automatic retries.

    host is the simulator's reachable host from the caller, not the vehicle ID.
    Construction performs no network I/O. Calls block for at most timeout seconds
    per HTTP operation; use outside flight/control callbacks.
    """

    def __init__(self, host='127.0.0.1', port=8991, *, timeout=3.0, scene_port=None):
        if not isinstance(host, str) or not host or any(c in host for c in '/?#@'):
            raise ValueError('host must be a hostname or IP address, without a URL scheme')
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError('port must be an integer in 1..65535')
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if scene_port is not None and (isinstance(scene_port, bool) or not isinstance(scene_port, int) or not 1 <= scene_port <= 65535):
            raise ValueError('scene_port must be an integer in 1..65535')
        address = f'[{host}]' if ':' in host and not host.startswith('[') else host
        self.url = f'http://{address}:{port}/aerohub/rpc'
        self.scene_url = None if scene_port is None else f'http://{address}:{scene_port}/aerohub/scene/rpc'
        self.timeout = timeout
        self._open = build_opener(ProxyHandler({})).open
        self._ids = count(1)

    def call(self, method, **params):
        """Low-level escape hatch for adapter bootstrap and future server methods."""
        return self._request(self.url, method, params)

    def call_scene(self, method, **params):
        """Call scene RPC without changing HUD, vehicle control or topic ownership."""
        if self.scene_url is None:
            raise ValueError('Scene RPC requires an explicit scene_port')
        return self._request(self.scene_url, method, params)

    def _request(self, url, method, params):
        request_id = next(self._ids)
        body = json.dumps(dict(jsonrpc='2.0', id=request_id, method=method, params=params),
                          ensure_ascii=False, allow_nan=False).encode('utf-8')
        request = Request(url, data=body, headers={'Content-Type': 'application/json'})
        try:
            with self._open(request, timeout=self.timeout) as reply:
                raw = reply.read()
        except (HTTPError, URLError, OSError, HTTPException) as error:
            raise ExtensionTransportError(f'{method} at {url}: {error}') from error
        try:
            response = json.loads(raw)
        except (ValueError, UnicodeError) as error:
            raise ExtensionProtocolError('Response is not valid JSON') from error
        if (not isinstance(response, dict) or response.get('jsonrpc') != '2.0'
                or type(response.get('id')) is not int or response['id'] != request_id
                or ('result' in response) == ('error' in response)):
            raise ExtensionProtocolError('Invalid JSON-RPC response envelope or request id')
        if 'error' in response:
            error = response['error']
            if (not isinstance(error, dict) or type(error.get('code')) is not int
                    or not isinstance(error.get('message'), str)):
                raise ExtensionProtocolError('Invalid JSON-RPC error')
            raise ExtensionRPCError(error['code'], error['message'], error.get('data'))
        return response['result']

    @staticmethod
    def _validate_actor(actor_id):
        if not isinstance(actor_id, str) or not actor_id.strip():
            raise ValueError('actor_id must be nonempty')

    def get_actor_state(self, actor_id='runner'):
        self._validate_actor(actor_id)
        return self.call_scene('GetActorState', actor_id=actor_id)

    def control_actor(self, actor_id, event):
        self._validate_actor(actor_id)
        if event not in ('start', 'stop', 'reset'):
            raise ValueError('actor event must be start, stop or reset')
        return self.call_scene('ControlActor', actor_id=actor_id, event=event)

    def start_actor(self, actor_id='runner'):
        return self.control_actor(actor_id, 'start')

    def stop_actor(self, actor_id='runner'):
        return self.control_actor(actor_id, 'stop')

    def reset_actor(self, actor_id='runner'):
        return self.control_actor(actor_id, 'reset')

    def set_title(self, text, *, side='left'):
        """Set one title; empty text clears only that side."""
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        return self.call('SetTitle', text=text, side=side)

    def show_message(self, text, *, duration_seconds=3.0):
        """Replace the center message; zero duration persists until cleared."""
        if (isinstance(duration_seconds, bool) or not isinstance(duration_seconds, (int, float))
                or not isfinite(duration_seconds) or not 0 <= duration_seconds <= 86400):
            raise ValueError('duration_seconds must be finite and in 0..86400')
        return self.call('ShowMessage', text=text, duration_seconds=duration_seconds)

    def clear_message(self):
        return self.call('ClearMessage')

    def get_state(self):
        """Read the current display model; this does not acknowledge rendered pixels."""
        return self.call('GetDisplayState')

    def start_run(self, vehicle_id):
        """Enter RUNNING and start a new timer/peak; repeated start preserves them."""
        return self.call('ControlRun', vehicle_id=vehicle_id, event='start')

    def finish_run(self, vehicle_id):
        """Enter FINISHED and freeze the timer; repeated finish preserves the result."""
        return self.call('ControlRun', vehicle_id=vehicle_id, event='finish')

    def reset_run(self, vehicle_id):
        """Enter IDLE and clear the timer and speed peak."""
        return self.call('ControlRun', vehicle_id=vehicle_id, event='reset')

    def set_run_state(self, vehicle_id, state):
        """Compatibility primitive; prefer start_run/finish_run/reset_run."""
        if state not in ('idle', 'running', 'finished'):
            raise ValueError('state must be idle, running or finished')
        return self.call('SetRunState', vehicle_id=vehicle_id, state=state)

    def control_timer(self, vehicle_id, event):
        """start/finish/reset one vehicle timer; reset also resets its speed peak."""
        if event not in ('start', 'finish', 'reset'):
            raise ValueError('event must be start, finish or reset')
        return self.call('ControlTimer', vehicle_id=vehicle_id, event=event)

    def set_view(self, vehicle_id, camera_id, image_type='RGB'):
        return self.call('SetView', vehicle_id=vehicle_id, camera_id=camera_id, image_type=image_type)

    def set_global_view(self):
        return self.call('SetView', vehicle_id=GLOBAL_VEHICLE_ID)


# Compatibility with the initial diagnostic adapter API: DisplayRPC(port).
class DisplayRPC(ProjectAirSimExtensionClient):
    def __init__(self, port=8991):
        super().__init__(port=port)
