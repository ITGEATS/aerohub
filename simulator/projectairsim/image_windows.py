"""Runtime image windows; native acquisition and subscriptions remain with callers."""
from dataclasses import dataclass
from threading import RLock
from typing import Mapping


@dataclass(frozen=True)
class ImageWindow:
    """A desktop window fed by an opaque source key (normally a native image topic)."""

    name: str
    source: str

    def __post_init__(self):
        if not self.name.strip() or not self.source.strip():
            raise ValueError('Window name and source must be nonempty')


def _display_worker(settings, entries, frames, stopped):
    # Spawned process owns OpenCV/Qt for this entire group's lifetime.
    from projectairsim.image_utils import ImageDisplay
    from queue import Empty
    display = ImageDisplay(**settings)
    for name, options in entries:
        display.add_image(name, **options)
    display.start()
    try:
        while not stopped.is_set() and display.running:
            try:
                name, image = frames.get(timeout=.1)
                display.receive(image, name)
            except Empty:
                pass
    finally:
        display.stop()


class _ProcessDisplay:
    """Bounded native GUI lifetime, isolated from Qt's thread affinity."""
    def __init__(self, **settings):
        import multiprocessing
        self._context = multiprocessing.get_context('spawn')
        self._settings = settings
        self._entries = []
        self._frames = self._context.Queue(maxsize=4)
        self._stopped = self._context.Event()
        self._process = None

    @property
    def running(self):
        return self._process is not None and self._process.is_alive()

    def add_image(self, name, **options):
        self._entries.append((name, options))

    def start(self):
        self._process = self._context.Process(
            target=_display_worker,
            args=(self._settings, self._entries, self._frames, self._stopped),
            daemon=True)
        self._process.start()

    def receive(self, image, name):
        from queue import Full
        if self.running:
            try:
                self._frames.put_nowait((name, image))
            except Full:
                pass  # Keep acquisition callbacks bounded when GUI is slower.

    def stop(self):
        self._stopped.set()
        if self._process is not None:
            self._process.join(timeout=2)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2)
            if self._process.is_alive():
                raise RuntimeError('Display worker did not exit')
            self._process.close()
            self._process = None
        self._frames.cancel_join_thread()
        self._frames.close()


def _native_display(**kwargs):
    return _ProcessDisplay(**kwargs)


class ImageWindows:
    """Own one group of native ImageDisplay windows in an isolated GUI process.

    Call show/close from the client control thread; receive may run in an image
    callback. Lists are registered before each display thread starts, avoiding
    live mutation of native window dictionaries. This object owns no connection.
    """

    def __init__(self, *, screen_size=(1920, 1080), window_size=(400, 225),
                 row_position=.8, display_factory=None):
        sw, sh = screen_size
        ww, wh = window_size
        if min(sw, sh, ww, wh) <= 0 or ww > sw or wh > sh:
            raise ValueError('Positive window dimensions must fit the screen')
        if not wh / (2 * sh) <= row_position <= 1 - wh / (2 * sh):
            raise ValueError('Window row must fit vertically on the screen')
        self._screen_size = screen_size
        self._window_size = window_size
        self._row_position = row_position
        self._factory = display_factory or _native_display
        self._lock = RLock()
        self._display = None
        self._windows = ()

    @property
    def windows(self):
        """Requested current list; windows appear only after receiving a frame."""
        with self._lock:
            return self._windows

    def show(self, windows):
        """Replace the whole group, preserving list order; an empty list closes it.

        Validate before touching the old group. Runtime construction/start errors
        leave the group closed and propagate to the caller for explicit retry.
        """
        windows = tuple(windows)
        if any(not isinstance(w, ImageWindow) for w in windows):
            raise TypeError('Expected ImageWindow entries')
        if len({w.name for w in windows}) != len(windows):
            raise ValueError('Window names must be unique')
        if len(windows) * self._window_size[0] > self._screen_size[0]:
            raise ValueError('Too many windows for one automatically arranged row')
        with self._lock:
            self.close()
            if not windows:
                return
            display = self._factory(
                num_subwin=len(windows),
                screen_res_x=self._screen_size[0], screen_res_y=self._screen_size[1],
                subwin_width=self._window_size[0], subwin_height=self._window_size[1],
                subwin_y_pct=self._row_position)
            try:
                for index, window in enumerate(windows):
                    display.add_image(window.name, subwin_idx=index,
                                      resize_x=self._window_size[0],
                                      resize_y=self._window_size[1])
                display.start()
            except Exception:
                display.stop()
                raise
            self._display = display
            self._windows = windows

    def receive(self, source: str, image: Mapping):
        """Route a native image to all matching windows; closed/unselected is ignored.

        Source keys must identify the actual stream, not a mutable window alias.
        No queue or display object is retained across a replacement.
        """
        with self._lock:
            if self._display is None or not self._display.running:
                return
            for window in self._windows:
                if window.source == source:
                    self._display.receive(image, window.name)

    def close(self):
        """Close only this group. Never reuse the stopped native display object."""
        with self._lock:
            if self._display is not None:
                self._display.stop()
                self._display = None
            self._windows = ()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
