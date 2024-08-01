from setuptools import setup, find_packages

package_name = "cartpole"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alvaro J. Gaona",
    maintainer_email="alvgaona@gmail.com",
    description="A cartpole control package for educational purposes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["control = cartpole.control:main"],
    },
)
