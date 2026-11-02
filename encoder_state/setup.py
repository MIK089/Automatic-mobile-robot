from setuptools import find_packages, setup

package_name = 'encoder_state'

setup(
    name='encoder_state',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='oyegokemichael89@gmail.com',
    description='Reads encoder state from serial port of a microcontroller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder_state = encoder_state.encoder_state:main'
        ],
    },
)
