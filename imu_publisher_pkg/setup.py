from setuptools import find_packages, setup

package_name = 'imu_publisher_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=[
        'imu_publisher_pkg.imu_publisher',
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-mpu6050'
    ],
    zip_safe=True,
    maintainer='epamachining',
    maintainer_email='calebsanfo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_publisher_pkg.imu_publisher:main',
        ],
    },
)
