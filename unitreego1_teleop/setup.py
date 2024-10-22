from setuptools import find_packages, setup

package_name = 'unitreego1_teleop'

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
    maintainer='ulises',
    maintainer_email='ulises@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'keyboard_press = unitreego1_teleop.keyboard_press:main',
        'my_teleop = unitreego1_teleop.my_teleop:main',
        'detect_ranges = unitreego1_teleop.detect_ranges:main',
        'teleop_ranges = unitreego1_teleop.teleop_ranges:main'
        ],
    },
)
