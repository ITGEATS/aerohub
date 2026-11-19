"""Bounded ROS service calls from the planner worker thread."""
from threading import Event


def call_inference(client, request, timeout):
    from .grounding_client import ClientError

    if not client.service_is_ready():
        raise ClientError('Inference service is not ready')
    future = client.call_async(request)
    done = Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        client.remove_pending_request(future)
        future.cancel()
        raise ClientError('Inference service timed out')
    if future.exception() is not None:
        raise ClientError(str(future.exception()))
    result = future.result()
    if not result.success:
        raise ClientError(f'{result.error_code}: {result.message}')
    return result
