import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'napoleon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     # install your worlds and models
     (os.path.join('share', package_name, 'worlds'),
      glob('worlds/*.sdf')),
     (os.path.join('share', package_name, 'models'),
      glob('models/**', recursive=True)),
 ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='marco_alemao@hotmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = napoleon.state_machine:main'
        ],
    },
)
