from setuptools import setup
from glob import glob

package_name = 'robot_web_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Templates are also shipped as package_data (below) so the running
        # process can find them next to the module; this share/ copy is for
        # inspection / ros2 pkg prefix discovery.
        ('share/' + package_name + '/templates', glob('robot_web_bridge/templates/*.html')),
        ('share/' + package_name + '/static', glob('robot_web_bridge/static/*')),
    ],
    include_package_data=True,
    package_data={package_name: ['templates/*.html', 'static/*', 'config/*.yaml']},
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'jinja2', 'pyyaml'],
    zip_safe=False,
    maintainer='BlueOC',
    maintainer_email='ivanchen.code@gmail.com',
    description='HTTP API + HTMX/Tailwind UI bridge for commanding the JetRacer over ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = robot_web_bridge.app:main',
        ],
    },
)
