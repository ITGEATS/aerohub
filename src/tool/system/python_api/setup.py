from setuptools import find_packages, setup
setup(name='uav_system_python_api', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_system_python_api']),('share/uav_system_python_api',['package.xml']), ('share/uav_system_python_api/modules', ['uav_system_python_api/system_module.py'])],
 install_requires=['setuptools'], license='Apache-2.0')
