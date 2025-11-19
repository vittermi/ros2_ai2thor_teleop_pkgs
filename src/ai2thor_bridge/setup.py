from setuptools import find_packages, setup

package_name = 'ai2thor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vittorio',
    maintainer_email='vittorio.ermini@studenti.unimi.it',
    description='Bridge components that allow ai2thor to communicate with ros2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_listener = ai2thor_bridge.teleop_listener:main',
        ],
    },
)
