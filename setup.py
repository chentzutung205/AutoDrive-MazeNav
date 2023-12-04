from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bb8_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tzu-Tung Chen',
    maintainer_email='tchen604@gatech.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'camera_testing=bb8_final.camera_testing:main',
        	'trained_knn=bb8_final.trained_knn:main',
        	'surrounding=bb8_final.surrounding:main',
        	'frontal_viewer=bb8_final.frontal_viewer:main',
        	'controller=bb8_final.controller:main',
        ],
    },
)
