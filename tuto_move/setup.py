from setuptools import setup

package_name = 'tuto_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='bot@mb6.imt-nord-europe.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_test = tuto_move.move_test:main',
            'move_1m = tuto_move.move:move_1m',
            'rear = tuto_move.move:rear',
            'turn_left = tuto_move.move:turn_left',
            'turn_right = tuto_move.move:turn_right',
            'scan_echo = tuto_move.scan_echo:main',
            'reactive_move = tuto_move.reactive_move:reactive_move'
        ],
    },
)
