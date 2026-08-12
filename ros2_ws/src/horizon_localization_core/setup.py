from setuptools import find_packages, setup

package_name = 'horizon_localization_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Horizon Team',
    maintainer_email='slam@gmail.com',
    description='Package for local relative tracking, local odometry, ArUco detection, and base sensor processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tag_detector = horizon_localization_core.aruco_tag_detector:main',
            'mock_zed_publisher = horizon_localization_core.mock_zed_publisher:main',
        ],
    },
)
