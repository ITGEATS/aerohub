"""Join callback workers before destroying ROS entities on the fixed Humble runtime."""


def shutdown_executor(executor, spin_thread=None, timeout_sec=5.0):
    if not executor.shutdown(timeout_sec=timeout_sec):
        raise TimeoutError('ROS callbacks did not finish before shutdown')
    if spin_thread is not None:
        spin_thread.join(timeout=timeout_sec)
        if spin_thread.is_alive():
            raise TimeoutError('ROS spin thread did not stop')
    # The pinned Humble executor stops spinning but does not join its Python
    # ThreadPoolExecutor. Queued subscription tasks can otherwise run after
    # destroy_node(). Keep this version-specific bridge in one place.
    executor._executor.shutdown(wait=True)
    for future in executor._futures:
        if future.done():
            future.result()  # Report errors; never hide unobserved exceptions.
