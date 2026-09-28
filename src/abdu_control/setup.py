from setuptools import find_packages, setup

package_name = "abdu_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/mission.yaml"]),
        (
            "share/" + package_name + "/data",
            ["abdu_control/data/orange_flare_model.npz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="ABDU ROV denetim gorevi ve 6-DOF teleop",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspect_mission = abdu_control.inspect_mission:main",
            "teleop = abdu_control.teleop:main",
            "train_orange_flare = abdu_control.train_orange_flare:main",
        ],
    },
)
