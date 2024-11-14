from setuptools import setup

package_name = 'sinrg_robot_sdk'
config_pkg = 'sinrg_robot_sdk/config'
board = 'sinrg_robot_sdk/board_manager'
servoController = 'sinrg_robot_sdk/servo_controller'
robot_data = 'sinrg_robot_sdk/robot_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, config_pkg, board, servoController, robot_data],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_manager=sinrg_robot_sdk.robot_controller_node:main'
            # 'robot_test = sinrg_robot_sdk.robot_test:main'
        ],
    },
)
