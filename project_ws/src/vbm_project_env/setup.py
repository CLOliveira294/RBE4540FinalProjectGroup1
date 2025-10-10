import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'vbm_project_env'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
         (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CristianO',
    maintainer_email='cloliveira@wpi.edu',
    description='Description',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test1 = vbm_project_env.test1:main',
        ],
    },
)
