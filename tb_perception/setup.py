from setuptools import find_packages, setup

package_name = 'tb_perception'

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
    description='Detection de personnes (detector_node : backends YOLO et couleur HSV)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = tb_perception.detector_node:main',
            'hsv_tuner = tb_perception.hsv_tuner:main',
        ],
    },
)
