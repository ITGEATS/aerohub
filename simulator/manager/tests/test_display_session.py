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

    def test_capture_only_rgb_can_supply_main_view_without_extra_capture(self):
        scene={'id':'Test','actors':[{'type':'robot','name':'Drone1','robot-config':{'sensors':[
            {'type':'camera','id':'FrontCamera','enabled':True,'capture-settings':[
                {'image-type':0,'streaming-enabled':False,'capture-enabled':True},
                {'image-type':1,'streaming-enabled':False,'capture-enabled':False}]},
            {'type':'camera','id':'Chase','enabled':False,'capture-settings':[
                {'image-type':0,'streaming-enabled':True,'capture-enabled':False}]}]}}]}
        self.assertEqual(views_from_scene(scene)[0],[])
        views,images=views_from_scene(scene,include_captured_views=True)
        self.assertEqual([(v['camera_id'],v['image_type'])for v in views],[('FrontCamera','RGB')])
        self.assertEqual(len(images),1)
        self.assertFalse(scene['actors'][0]['robot-config']['sensors'][0]['capture-settings'][0]['streaming-enabled'])

    def test_unexpanded_robot_fails_without_guessing(self):
        with self.assertRaisesRegex(ValueError,'expanded'):
            views_from_scene({'actors':[{'type':'robot','robot-config':'robot.jsonc'}]})
