from setuptools import setup, find_packages
setup(name='uav_target_tracking', version='0.4.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_target_tracking']), ('share/uav_target_tracking', ['package.xml']), ('share/uav_target_tracking/upstream', ['../../../../deployment/airsim/demos/target_tracking/reference/yopo_tracker/YOPO/policy/target_ekf.py', '../../../../deployment/airsim/demos/target_tracking/reference/yopo_tracker/LICENSE'])],
 install_requires=['setuptools'], license='MIT',
 entry_points={'console_scripts': ['target_tracking = uav_target_tracking.node:main']})
