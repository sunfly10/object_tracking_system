from setuptools import find_packages, setup

package_name = 'flask_camera_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/flask_camera_serial.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ysh',
    maintainer_email='rysh0708@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_camera=flask_camera_serial.flask_camera:main',
            'angle_calculation_transmission=flask_camera_serial.angle_calculation_transmission:main',
            'name_number_transmission=flask_camera_serial.name_number_transmission:main'
        ],
    },
)
