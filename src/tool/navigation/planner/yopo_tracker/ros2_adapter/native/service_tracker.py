"""Keep the pinned tracking callbacks local and delegate only the network forward pass."""

from array import array
import os
import sys
from threading import Event, Lock
from uuid import uuid4

from .tracker_model import WEIGHT_SHA256


def load_service_tracker(upstream):
    # Upstream lattice and state transforms choose their device at construction.
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    sys.path.insert(0, str(upstream.resolve() / 'YOPO'))
    import torch
    from config.config import cfg
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from scipy.spatial.transform import Rotation
    from sensor_msgs.msg import Image
    from test_yopo_ros import YopoNet
    from policy.primitive import LatticePrimitive
    from policy.state_transform import StateTransform
    from uav_inference_interfaces.srv import InferYopoTracker
    from visualization_msgs.msg import MarkerArray

    class ServiceTracker(YopoNet):
        def __init__(self):
            Node.__init__(self, 'yopo_tracker_native_state')
            cfg['train'] = False
            self.height, self.width = cfg['image_height'], cfg['image_width']
            self.min_dis, self.max_dis = .04, 20.
            self.simulation = self.plan_from_reference = True
            self.use_trt = self.verbose = self.visualize = False
            self.conf_thresh, self.ekf_thresh = .9, 2.
            self.Rotation_bc = Rotation.identity().as_matrix()
            self.device = 'cpu'
            self.ctrl_dt, self.ekf_dt = .02, 1 / 30
            self.search_dist, self.search_yaw_rate, self.search_yaw_acc, self.brake_acc = 3., 3.14, 3., 4.5
            self.hover_window, self.hover_speed, self.hover_u_move, self.hover_target_move = 1., .4, 20., .8
            self.lock = Lock()
            self.state_transform = StateTransform()
            self.lattice_primitive = LatticePrimitive.get_instance()
            self.reset_state()
            self.target_pub = self.create_publisher(PoseStamped, '/yopo_net/target_pred', 1)
            self.ekf_pub = self.create_publisher(PoseStamped, '/yopo_net/target_ekf', 1)
            self.target_image_pub = self.create_publisher(Image, '/yopo_net/target_image', 1)
            self.best_traj_pub = self.create_publisher(MarkerArray, '/yopo_net/best_traj_visual', 1)
            self.inference_client = self.create_client(InferYopoTracker, '/drone0/inference/yopo_tracker/infer')
            self.policy = self.infer
            self.pending_inference = None

        def inference_ready(self):
            return self.inference_client.service_is_ready() and (
                self.pending_inference is None or self.pending_inference.done())

        def callback_images(self, depth, rgb):
            self.rgbd_headers = rgb.header, depth.header, self.odom.header
            return super().callback_images(depth, rgb)

        def infer(self, image, observation):
            if not self.inference_ready():
                raise RuntimeError('YOPO-Tracker inference is unavailable or still handling the previous request')
            request = InferYopoTracker.Request(request_id=uuid4().hex)
            request.rgb_header, request.depth_header, request.odometry_header = self.rgbd_headers
            request.rgbd = array('B', image.numpy().astype('<f4', copy=False).tobytes())
            request.observation = observation.flatten().tolist()
            future = self.inference_client.call_async(request)
            self.pending_inference = future
            completed = Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(2.5):
                raise TimeoutError('YOPO-Tracker ROS inference response timed out')
            response = future.result()
            if not response.success:
                raise RuntimeError(f'{response.error_code}: {response.message}')
            if (response.request_id != request.request_id or
                    (response.rgb_header, response.depth_header, response.odometry_header) != self.rgbd_headers or
                    response.weight_sha256 != WEIGHT_SHA256 or response.device != 'cuda'):
                raise RuntimeError('Mismatched YOPO-Tracker inference response')
            return tuple(torch.from_numpy(values.copy()).reshape(shape) for values, shape in (
                (response.endstate, (1, 9, 3, 5)), (response.target, (1, 3, 3, 5)),
                (response.cost, (1, 3, 5)), (response.score, (1, 3, 5)), (response.duration, (1, 3, 5))))

    return ServiceTracker(), WEIGHT_SHA256
