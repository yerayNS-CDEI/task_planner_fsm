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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ynavarro@cdei.upc.edu',
    maintainer_email='yeray.navarro@estudiantat.upc.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = task_planner_fsm.fsm_node:main',
            'mock_server = task_planner_fsm.mock_server:main',
        ],
    },
)
