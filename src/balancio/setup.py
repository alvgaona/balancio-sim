from setuptools import find_packages, setup

package_name = "balancio"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alvaro J. Gaona",
    maintainer_email="alvgaona@gmail.com",
    description="The Balancio Sim package to develop applications with NVIDIA Isaac Sim",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["pid = balancio.pid:main"],
    },
)
