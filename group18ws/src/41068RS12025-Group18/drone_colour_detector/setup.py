from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'drone_colour_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group18',
    maintainer_email='group18@example.com',
    description='HSV color-based tree detection for drone aerial surveillance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tree_detector = drone_colour_detector.tree_detector:main',
            'hsv_calibrator = drone_colour_detector.hsv_calibrator:main',
        ],
    },
)
