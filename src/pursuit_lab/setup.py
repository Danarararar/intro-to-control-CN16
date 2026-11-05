from setuptools import setup

package_name = 'pursuit_lab'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/params.yaml', 'config/pursuit.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasyaarya',
    maintainer_email='rasyaarya@users.noreply.github.com',
    description='Pure Pursuit untuk lab hands-on.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pursuit_node = pursuit_lab.pursuit_node:main',
        ],
    },
)
