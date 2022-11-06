from setuptools import setup

package_name = 'rumicar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrei',
    maintainer_email='akela1101@gmail.com',
    description='RumiCar ROS2 driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rumicar = rumicar.rumicar:main',
        ],
    },
)
