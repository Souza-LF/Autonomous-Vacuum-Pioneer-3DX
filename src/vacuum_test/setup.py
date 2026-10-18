from setuptools import find_packages, setup

package_name = 'vacuum_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/behavior_trees',
            ['behavior_trees/coverage_through_poses.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luis',
    maintainer_email='luis@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'safety_distance = vacuum_test.safety_distance:main',
		'wall_follower = vacuum_test.wall_follower_node:main',
		'boustrophedon_planner = vacuum_test.boustrophedon_planner:main',
        ],
    },
)
