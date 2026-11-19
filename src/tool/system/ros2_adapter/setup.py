from setuptools import find_packages, setup
setup(name='uav_system_tool', version='0.1.0', packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/uav_system_tool']),('share/uav_system_tool',['package.xml'])],
 install_requires=['setuptools'], license='Apache-2.0', entry_points={'console_scripts':['system_tool = uav_system_tool.node:main']})
