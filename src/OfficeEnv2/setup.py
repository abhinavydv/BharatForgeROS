from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'OfficeEnv2'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob("launch/*")),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'OfficeEnv2'), glob(os.path.join('OfficeEnv2', '**/**'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**/**'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='abhinavyadavdev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tf_publisher = OfficeEnv2.tf_publisher:main",
            "swarm_to_one_bot = OfficeEnv2.swarm_to_one_bot:main",
        ],
    },
)
