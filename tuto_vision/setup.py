from setuptools import setup

package_name = 'tuto_vision'

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
            'simple_request = tuto_vision.simple_request:main',
            'camera_image = tuto_vision.camera_image:main',
            'vision = tuto_vision.vision:template_matching',
            'take_picture = tuto_vision.take_picture:photo'
        ],
    },
)
