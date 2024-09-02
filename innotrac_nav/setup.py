from setuptools import find_packages, setup

package_name = 'innotrac_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-iron',
    maintainer_email='krishna8695@gmail.com',
    description='InnoTRAC navigation stack',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'name = package_name.name:main'
        ],
    },
)
