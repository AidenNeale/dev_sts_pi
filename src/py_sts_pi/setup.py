from setuptools import setup

package_name = 'py_sts_pi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aiden Neale',
    maintainer_email='arn519@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sts_pi_camera = py_sts_pi.APIs.sts_pi_camera:main',
            'display = py_sts_pi.display:main',
            'aruco = py_sts_pi.arUco:main',
            'motors = py_sts_pi.APIs.motor_control:main',
            'simpub = py_sts_pi.simpub:main'
        ],
    },
)
