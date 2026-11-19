"""Actual callback seam; no model construction or synthetic flight claim."""
import threading
from types import SimpleNamespace as Obj
import unittest
from sensor_msgs.msg import Image
from uav_object_detection.node import ObjectDetectionTool
from uav_object_detection.snapshot import SnapshotAdapter


class LatestFrameTests(unittest.TestCase):
    def node(self):
        node=Obj(lock=threading.RLock(),enabled=True,generation=1,last_rgb=0.,
            pending=None,dropped=0,rejected=0,frame_available=threading.Event(),streaming=True)
        node.trace=lambda *args,**kwargs:None
        node.get_clock=lambda:Obj(now=lambda:Obj(nanoseconds=100*10**9))
        node.on_rgb=lambda m:ObjectDetectionTool.on_rgb(node,m)
        return node

    def image(self,stamp):
        msg=Image();msg.header.stamp.sec=stamp;return msg

    def test_slow_worker_does_not_block_capture_and_latest_overwrites(self):
        node=self.node()
        # A separate inference lock may be held throughout these callbacks.
        node.call_lock=threading.Lock();node.call_lock.acquire()
        for i in range(1,6):node.on_rgb(self.image(i))
        self.assertEqual(node.pending[1].header.stamp.sec,5)
        self.assertEqual(node.dropped,4)
        self.assertTrue(node.frame_available.is_set())
        node.call_lock.release()

    def test_single_camera_callback_fans_out_without_snapshot_wait(self):
        node=self.node();adapter=SnapshotAdapter.__new__(SnapshotAdapter)
        adapter.node=node;adapter.condition=threading.Condition()
        adapter.last_stamp_ns=0;adapter.waiting=False;adapter.image=None
        image=self.image(1)
        adapter.on_image(image)
        self.assertIs(node.pending[1],image)
        self.assertIsNone(adapter.image)
        adapter.on_image(image)
        self.assertEqual(node.dropped,0)  # duplicate never queued twice


if __name__=='__main__':unittest.main()
