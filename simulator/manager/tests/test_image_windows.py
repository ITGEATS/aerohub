"""Group ownership and source routing; native GUI behavior is tested separately."""
import unittest
from simulator.projectairsim.image_windows import ImageWindow, ImageWindows


class Display:
    def __init__(self, **kwargs):
        self.running = False
        self.events = []

    def add_image(self, name, **kwargs):
        self.events.append(('add', name, kwargs['subwin_idx']))

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def receive(self, image, name):
        self.events.append(('frame', name, image))


class ImageWindowsTest(unittest.TestCase):
    def test_replacement_closes_old_group_and_drops_old_source(self):
        instances = []
        def factory(**kwargs):
            instance = Display(**kwargs)
            instances.append(instance)
            return instance
        with ImageWindows(display_factory=factory) as group:
            group.receive('rgb', {'data': b'before-open'})
            group.show([ImageWindow('RGB', 'rgb'), ImageWindow('Depth', 'depth')])
            old = instances[-1]
            self.assertEqual(old.events[:2], [('add', 'RGB', 0), ('add', 'Depth', 1)])
            group.receive('rgb', {'data': b'old'})
            group.show([ImageWindow('RGB', 'depth')])
            self.assertFalse(old.running)
            new = instances[-1]
            group.receive('rgb', {'data': b'stale'})
            self.assertEqual(new.events, [('add', 'RGB', 0)])
            group.receive('depth', {'data': b'new'})
            self.assertEqual(new.events[-1], ('frame', 'RGB', {'data': b'new'}))
            group.show([])
            self.assertFalse(new.running)
            group.show([ImageWindow('RGB', 'rgb')])
            self.assertIsNot(instances[-1], old)
        self.assertFalse(instances[-1].running)

    def test_invalid_replacement_preserves_current_group(self):
        group = ImageWindows(display_factory=Display)
        first = ImageWindow('RGB', 'rgb')
        group.show([first])
        with self.assertRaises(ValueError):
            group.show([first, first])
        self.assertEqual(group.windows, (first,))
        with self.assertRaises(ValueError):
            group.show([ImageWindow(str(i), 'rgb') for i in range(5)])
        self.assertEqual(group.windows, (first,))
        group.close()


if __name__ == '__main__':
    unittest.main()
