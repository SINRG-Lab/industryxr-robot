from setuptools import setup

package_name = 'sinrg_servo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'sinrg_servo_subscriber = sinrg_servo_controller.servo_controller_sub_node:main',
            'sinrg_servo_publisher = sinrg_servo_controller.servo_controller_pub_node:main'
        ],
    },
)
