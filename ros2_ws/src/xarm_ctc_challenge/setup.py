from setuptools import find_packages, setup

package_name = 'xarm_ctc_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/challenge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='CTC vs PD/PID challenge — xArm Lite 6 task-space tracking via weighted IK',
    license='TODO',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'challenge_controller  = xarm_ctc_challenge.controller_node:main',
            'challenge_analysis    = xarm_ctc_challenge.analysis:main',
            'go_home               = xarm_ctc_challenge.go_home:main',
            'joint_state_logger    = xarm_ctc_challenge.joint_state_logger:main',
        ],
    },
)
