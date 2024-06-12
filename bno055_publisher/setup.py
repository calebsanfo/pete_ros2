from setuptools import find_packages, setup

package_name = 'bno055_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=['bno055_publisher.bno055_publisher_node'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epamachining',
    maintainer_email='calebsanfo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_publisher_node = bno055_publisher.bno055_publisher_node:main',
        ],
    },

)
