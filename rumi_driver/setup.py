from setuptools import setup
from glob import glob

package_name = 'rumi_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrei',
    maintainer_email='akela1101@gmail.com',
    description='RumiCar ROS2 driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rumi_driver = rumi_driver.rumi_driver:main',
        ],
    },
)
