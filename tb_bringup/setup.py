import os

from setuptools import find_packages, setup

package_name = 'tb_bringup'


def tree(src):
    """Installe un dossier en conservant son arborescence (models/, worlds/...)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + tree('launch') + tree('config') + tree('worlds') + tree('urdf') + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Amine Herch',
    maintainer_email='herch4515@gmail.com',
    description='Launch files, configs sim/real, monde Gazebo avec personne et modele burger_cam',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_joystick = tb_bringup.ball_joystick:main',
        ],
    },
)
