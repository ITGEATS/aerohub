from setuptools import find_packages, setup


setup(
    name="uav_falcon_adapter",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/uav_falcon_adapter"]),
        ("share/uav_falcon_adapter", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["falcon_adapter = uav_falcon_adapter.node:main"]
    },
)
