from setuptools import setup

package_name = 'my_package'
submodules = "my_package/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mibum',
    maintainer_email='minh-anh.tran@alumni.fh-aachen.de',
    description='my package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'my_node = my_package.my_node:main',
        'my_sensor = my_package.sensor_node:main',
        'my_scanner = my_package.scanner_node:main', 
        'my_lane = my_package.lane_following:main',
        'my_moveit = my_package.moveit_node:main',
        'my_test = my_package.can_frame:main',
        'my_nav = my_package.nav_node:main'
        ],
    },
)
