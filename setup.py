from setuptools import find_packages, setup

package_name = 'task_planner_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # The vendored sensor pipelines (task_planner_fsm/sensors/vendor) ship their
    # configs and docs next to the code; model weights and runtime data live in
    # models/ and data/ at the package root and are deliberately NOT installed
    # (see models/README.md, data/README.md). Only matters for a non-symlink
    # install; with --symlink-install the source tree is used in place.
    package_data={
        'task_planner_fsm.sensors': ['vendor/VERSIONS.md'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline': ['*.md'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline.GPRTools': ['*.md'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline.Hyperbola_Segmentation': ['*.md', '*.txt'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline.Hyperbola_Segmentation.gpr_integration': ['*.yaml'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline.Hyperbola_Segmentation.tahzeeb_original': ['*.yaml', '*.md'],
        'task_planner_fsm.sensors.vendor.gpr_pipeline.Line_Segmentation': ['*.yaml', '*.md', '*.txt'],
        'task_planner_fsm.sensors.vendor.hsi_pipeline': ['*.md', '*.txt'],
        'task_planner_fsm.sensors.vendor.hsi_pipeline.hsi_integration': ['*.json'],
        'task_planner_fsm.sensors.vendor.pokeye_decision_pipeline': ['*.md', '*.txt'],
        'task_planner_fsm.sensors.vendor.pokeye_decision_pipeline.pokeye_decision': ['*.json'],
        'task_planner_fsm.sensors.vendor.pokeye_decision_pipeline.examples': ['*.json'],
        'task_planner_fsm.sensors.vendor.pokeye_decision_pipeline.example_outputs': ['*.json'],
    },
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
        'scripts/hyperspectral_bench',
        'scripts/hyperspectral_wall_test',
        'scripts/check_sensor_setup',
        'scripts/process_sensor_session',
    ],
)
