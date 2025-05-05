from setuptools import find_packages, setup

package_name = 'kinova_gen3_lite_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/empty_world.sdf']),
        ('share/' + package_name + '/urdf', ['urdf/gen3_lite.urdf']),
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
            "auto_data_collector = kinova_gen3_lite_sim.auto_data_collector:main",
        ],
    },
)
