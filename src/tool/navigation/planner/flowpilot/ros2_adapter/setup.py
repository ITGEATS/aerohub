from setuptools import setup, find_packages
setup(name='uav_flowpilot_planner', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_flowpilot_planner']),
                  ('share/uav_flowpilot_planner', ['package.xml', 'LICENSE'])],
      install_requires=['setuptools'], maintainer='Aerohub', maintainer_email='dev@example.com',
      description='FlowPilot reference-mode single vehicle planner', license='MIT',
      entry_points={'console_scripts': ['flowpilot_planner = uav_flowpilot_planner.node:main']})
