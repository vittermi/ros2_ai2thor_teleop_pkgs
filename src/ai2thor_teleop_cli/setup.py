from setuptools import find_packages, setup

package_name = 'ai2thor_teleop_cli'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ai2thor_msgs'],
    zip_safe=True,
    maintainer='vittorio',
    maintainer_email='vittorio.ermini@studenti.unimi.it',
    description='Keyboard teleop that publishes ai2thor agent intents',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_key = ai2thor_teleop_cli.teleop_key:main',
        ],
    },
)
