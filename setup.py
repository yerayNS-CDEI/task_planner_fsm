from setuptools import find_packages, setup

package_name = 'task_planner_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeray',
    maintainer_email='yeray.navarro@upc.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/fsm_node',
        'scripts/mock_server',
        'scripts/goal_status_listener',
    ],
)
