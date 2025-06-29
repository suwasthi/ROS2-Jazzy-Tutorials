from setuptools import find_packages, setup

package_name = 'core_concepts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task5_launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suwasthi',
    maintainer_email='suwasthi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'pose_re_pub = core_concepts.pose_re_pub:main',
        'params_setter = core_concepts.params_setter:main',
        'twist_from_database = core_concepts.twist_from_database:main',
        'zero_twist = core_concepts.zero_twist:main',
        'param_reader = core_concepts.param_reader:main',


        # Add other nodes here later
    ],
    },
    package_data={
    'core_concepts': ['values.csv'],
    },

)
