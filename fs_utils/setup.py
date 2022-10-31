from setuptools import setup

package_name = 'fs_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "numpy"],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='tituszban@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "faux_lidar = fs_utils.faux_lidar:main",
            "fuzzing = fs_utils.fuzzing.__main__:main"
        ],
    },
)
