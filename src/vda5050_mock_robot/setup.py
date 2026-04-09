from setuptools import setup

package_name = 'vda5050_mock_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/mock_robot.yaml']),
        ('share/' + package_name + '/launch', ['launch/mock_robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='Mock VDA5050 robot for testing the rmf_vda5050_adapter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_robot = vda5050_mock_robot.main:main',
        ],
    },
)
