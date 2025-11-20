from setuptools import find_packages, setup

package_name = 'boustrophedon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lewisjs',
    maintainer_email='lewisjs4@cse.sc.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Run with: ros2 run boustrophedon main
            'main = boustrophedon.main:main',
        ],
    },
)
