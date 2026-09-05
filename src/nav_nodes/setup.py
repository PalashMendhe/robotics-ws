from setuptools import find_packages, setup

package_name = 'nav_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jade',
    maintainer_email='palashmendhe777@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'broadcaster_node    = nav_nodes.broadcaster_node:main',
        'aruco_detector_node = nav_nodes.aruco_detector_node:main',
        'arm_controller_node = nav_nodes.arm_controller_node:main',
        'docking_node        = nav_nodes.docking_node:main',
    ],
    },
)
