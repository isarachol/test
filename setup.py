import os
from glob import glob
from setuptools import setup
from setuptools import find_packages, setup

package_name = 'bearing_formation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  	    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
  	    (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tand',
    maintainer_email='tand.isara2001@gmail.com',
    description='package for simulating formation control using bearing only information in 2D planar space',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_state_publisher = bearing_formation_control.multi_state_publisher:main',
            'x_multi_state_publisher = bearing_formation_control.x_multi_state_publisher:main',
        ],
    },
)
