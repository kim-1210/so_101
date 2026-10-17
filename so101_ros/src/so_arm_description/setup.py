from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'so_arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share/", package_name, "urdf"),
            glob("urdf/*.urdf") + glob("urdf/*.xacro"),
        ),
        (
            os.path.join("share/", package_name, "urdf", "so101_arm"),
            glob("urdf/so101_arm/*.usd"),
        ),
        (
            os.path.join("share/", package_name, "urdf", "so101_arm", "configuration"),
            glob("urdf/so101_arm/configuration/*"),
        ),
        (os.path.join("share/", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share/", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share/", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share/", package_name, "config"), glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='zxmzxm1324@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
