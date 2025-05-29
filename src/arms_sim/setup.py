from setuptools import find_packages, setup
from glob import glob

package_name = 'arms_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        # Include all files from the world directory
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        # Include all files from the urdf directory
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        # Include all files from the config directory
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab',
    maintainer_email='johanugandonou@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "auto_motion_explorer = arms_sim.auto_motion_explorer:main",
            "image_saver = arms_sim.image_saver:main",
            "robot_info_extractor = arms_sim.robot_info_extractor:main",
        ],
    },
)
