from setuptools import setup, find_packages
from glob import glob

package_name = 'soft_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['soft_gripper*']), 
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='your_email@example.com',
    description='Soft gripper control package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = soft_gripper.main:main',
        ],
    },
)








