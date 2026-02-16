from setuptools import setup

package_name = 'gps_lab1'

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
    maintainer='phillip',
    maintainer_email='muvadi.p@northeastern.edu',
    description='GPS serial to UTM publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_serial_to_utm = gps_lab1.gps_serial_to_utm:main',
        ],
    },
)
