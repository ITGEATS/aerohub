from setuptools import setup, find_packages
setup(name='uav_target_tracking_python_api', version='0.2.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_target_tracking_python_api']), ('share/uav_target_tracking_python_api', ['package.xml']), ('share/uav_target_tracking_python_api/modules', ['uav_target_tracking_python_api/target_tracking_module.py'])],
 install_requires=['setuptools'], license='MIT',
 entry_points={})
