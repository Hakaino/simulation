import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'napoleon'

def package_files(directory):
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [full_path]))
    return paths

# Static data files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
]

# Dynamically add models (preserve subdirectories)
data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
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
