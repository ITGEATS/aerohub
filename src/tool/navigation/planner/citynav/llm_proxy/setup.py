from setuptools import setup

package_name = "uav_citynav_llm_proxy"

setup(
    name=package_name, version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["citynav_llm_proxy = uav_citynav_llm_proxy.node:main"]},
)
