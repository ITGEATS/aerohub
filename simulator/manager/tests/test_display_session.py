import unittest
from simulator.projectairsim.display_session import views_from_scene


class ViewManifestTest(unittest.TestCase):
    def test_streaming_and_capture_are_independent(self):
        scene = {'id':'Test', 'actors':[{'type':'robot','name':'Drone1','robot-config':{
            'sensors':[{'type':'camera','id':'FrontCamera','enabled':True,'capture-settings':[
                {'image-type':0,'streaming-enabled':True,'capture-enabled':True},
                {'image-type':2,'streaming-enabled':False,'capture-enabled':True},
                {'image-type':4,'streaming-enabled':True,'capture-enabled':False}]}]}}]}
        views, images = views_from_scene(scene)
        self.assertEqual([v['image_type'] for v in views], ['RGB','DepthVis'])
        self.assertEqual([v['component'] for v in views], ['SceneCaptureComponent','DepthVisCaptureComponent'])
        self.assertEqual(len(images),2)
        self.assertTrue(images[1].source.endswith('/depth_camera'))

    def test_unexpanded_robot_fails_without_guessing(self):
        with self.assertRaisesRegex(ValueError,'expanded'):
            views_from_scene({'actors':[{'type':'robot','robot-config':'robot.jsonc'}]})
