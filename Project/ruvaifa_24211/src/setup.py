from setuptools import find_packages, setup

package_name = 'robot_conga'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Conga Team',
    maintainer_email='dev@robotconga.org',
    description='Robot Conga: A Leader-Follower Walking Approach to Sequential Path Following in Multi-Agent Systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
