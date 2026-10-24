from setuptools import setup

setup(
    name='wheel_odometry',
    version='0.1.0',
    packages=['wheel_odometry'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/wheel_odometry']),
        ('share/wheel_odometry', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yamana',
    maintainer_email='yamana@example.com',
    description='Planar omni wheel encoder odometry',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'encoder_odometry = wheel_odometry.node:main',
    ]},
)
