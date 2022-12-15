from setuptools import setup

package_name = 'ros_cap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        'launch/duckie_core.xml',
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duckiebot',
    maintainer_email='raimundo.lorca.c@ug.uchile.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
