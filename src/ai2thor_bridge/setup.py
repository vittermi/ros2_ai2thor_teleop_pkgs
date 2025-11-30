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
        ('share/ai2thor_bridge/launch', ['launch/ai2thor_tf_depth_bridge.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'redis',
    ],
    zip_safe=True,
    maintainer='-',
    maintainer_email='-',
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
            'odom_adapter = ai2thor_bridge.odom_adapter:main',
            'depth_image_adapter = ai2thor_bridge.depth_image_adapter:main',
        ],
    },

)
