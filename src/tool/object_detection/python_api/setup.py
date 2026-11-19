from setuptools import find_packages, setup

setup(name='uav_object_detection_python_api', version='0.4.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_object_detection_python_api']),
                  ('share/uav_object_detection_python_api', ['package.xml']),
                  ('share/uav_object_detection_python_api/modules',
                   ['uav_object_detection_python_api/object_detection_module.py'])],
      install_requires=['setuptools'], license='Apache-2.0')
