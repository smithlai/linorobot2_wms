import os
from setuptools import setup
from glob import glob

package_name = 'myautomapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['myautomapping.robot_navigator_tst'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usi',
    maintainer_email='oliverliu999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'mappinggoal = myautomapping.MappingGoal:main'
        ],
    },
)
