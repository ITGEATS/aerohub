from setuptools import find_packages, setup
from glob import glob

name = 'uav_elastic_tracker_adapter'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + name + '/config', glob('config/*.yaml') + glob('../../config/*.yaml'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='AeroHub', maintainer_email='maintainers@example.com',
      description='Elastic ROS 2 native RPC and standalone timed-path execution',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'elastic_tracker_native_rpc = ' + name + '.native_rpc:main',
          'elastic_tracker_native_node = ' + name + '.native_execution:main',
          'elastic_tracker_input_node = ' + name + '.input_node:main',
          'elastic_following = ' + name + '.following:main']})
