from setuptools import setup

package_name = 'py_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='harisankar995@gmail.com',
    description='Minimal WayPoint Publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = py_publisher.waypoint_publisher:main'
        ],
    },
)
