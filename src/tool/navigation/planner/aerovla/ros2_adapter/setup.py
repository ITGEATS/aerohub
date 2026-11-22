from setuptools import setup, find_packages
setup(name='uav_aerovla_planner', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_aerovla_planner']),
                  ('share/uav_aerovla_planner', ['package.xml'])],
      install_requires=['setuptools'], maintainer='Aerohub', maintainer_email='dev@example.com',
      description='AeroVLA VLA policy planner (Tool layer)', license='Apache-2.0',
      entry_points={'console_scripts': ['aerovla_planner = uav_aerovla_planner.planner_node:main',
          'aerovla_goto = uav_aerovla_planner.goto_node:main']})
