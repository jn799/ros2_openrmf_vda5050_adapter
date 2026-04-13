from setuptools import setup

package_name = 'vda5050_nav2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/nav2_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='VDA5050 to Nav2 bridge for TurtleBot3 integration testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nav2_bridge = vda5050_nav2_bridge.main:main',
        ],
    },
)
