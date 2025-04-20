import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'napoleon'
here = os.path.abspath(os.path.dirname(__file__))
# helper to collect all files under a directory
def collect_files(root_dir):
    paths = []
    for dirpath, dirnames, filenames in os.walk(root_dir):
        for f in filenames:
            paths.append(os.path.join(dirpath, f))
    return paths

setup(
     name=package_name,
     version='0.0.0',
     packages=find_packages(exclude=['test']),
     data_files=[
         ('share/ament_index/resource_index/packages',
          ['resource/' + package_name]),
         ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'),
          glob('launch/*_launch.py')),
         (os.path.join('share', package_name, 'worlds'),
          glob('worlds/*.sdf') + glob('worlds/*.world'))
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
