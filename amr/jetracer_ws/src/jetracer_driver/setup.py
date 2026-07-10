from setuptools import setup

package_name = 'jetracer_driver'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'jetracer_driver = jetracer_driver.cmd_vel_to_serial:main',
        ],
    },
)
