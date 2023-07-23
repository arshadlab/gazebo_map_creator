from setuptools import setup

package_name = 'gazebo_map_creator_ign'

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
    maintainer='Arshad Mehmood',
    maintainer_email='arshad.mehmood@intel.com',
    description='Scripts to send request for gazebo map creation',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'request_map = gazebo_map_creator_ign.request_map:main'
        ],
    },
)
