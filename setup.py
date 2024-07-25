from setuptools import find_packages, setup

package_name = 'my_pointcloud'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anel-b',
    maintainer_email='anybajrektarevic@gmail.com',
    description='Point cloud visualization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pc_publisher = my_pointcloud.pc_publisher:main",
            "pc_subscriber = my_pointcloud.pc_subscriber:main"
        ],
    },
)
