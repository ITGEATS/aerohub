from setuptools import find_packages, setup
setup(name='uav_motion_resource', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_motion_resource']),('share/uav_motion_resource',['package.xml'])],
 install_requires=['setuptools'], license='Apache-2.0')
