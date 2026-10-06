from setuptools import find_packages, setup

package_name = 'tb_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Amine Herch',
    maintainer_email='herch4515@gmail.com',
    description='Tracking SORT from scratch (lib sans dependance ROS), tracker_node, target_selector_node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_selector_node = tb_tracking.target_selector_node:main',
        ],
    },
)
