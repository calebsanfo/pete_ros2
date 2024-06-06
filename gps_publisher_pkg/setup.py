from setuptools import setup

package_name = 'gps_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'gps_publisher_pkg.gps_publisher',
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-gps'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 GPS Publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_publisher = gps_publisher_pkg.gps_publisher:main',
        ],
    },
)
