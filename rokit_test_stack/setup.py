
import glob
import os
from setuptools import setup

package_name = 'rokit_test_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, "rokit_test_stack/calculator", "rokit_test_stack/database"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rokit_test_stack/launch',
            glob.glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='annameer',
    maintainer_email='anna-maria.meer@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rokit_calculator = rokit_test_stack.main_calculator:main',
        ],
    },
)
