from setuptools import setup

package_name = 'robot_legion_teleop_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Ament index so ROS 2 can discover this package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vitruvian Systems LLC',
    maintainer_email='emilio@viturvian.systems',
    description='Keyboard teleop node modeled after teleop_twist_keyboard, with custom key bindings.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run robot_legion_teleop_python legion_teleop_key
            # will execute main() in teleop_legion_key.py
            'legion_teleop_key = robot_legion_teleop_python.teleop_legion_key:main',
        ],
    },
)
