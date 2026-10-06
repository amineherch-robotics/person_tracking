from setuptools import find_packages, setup

package_name = 'tb_control'

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
    description='Asservissement visuel (follower_controller) et watchdog de securite',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'follower_controller = tb_control.follower_controller:main',
            'estop_keyboard = tb_control.estop_keyboard:main',
        ],
    },
)
