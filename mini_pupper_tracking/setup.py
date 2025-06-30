from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_pupper_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    
    # Data files installation
    data_files=[
        # Required ROS package index files
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        
        # Model files
        (os.path.join('share', package_name, 'models'),
         glob(os.path.join('models', '*.onnx'))),
        
        # Launch files (NEW - critical addition)
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    
    # Package settings
    install_requires=['setuptools'],
    zip_safe=False,  # Required for ROS 2 launch files
    
    # Metadata
    maintainer='kishan',
    maintainer_email='kishangrewal06@gmail.com',
    description='Mini Pupper vision-based tracking system',
    license='Apache 2.0',
    tests_require=['pytest'],
    
    # Executable scripts
    entry_points={
        'console_scripts': [
            'main = mini_pupper_tracking.main:main',
        ],
    },
)