from setuptools import setup

package_name = 'command'

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
    maintainer='clem-irwt',
    maintainer_email='pene.clement@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'algoTwoDof = command.two_dof_com:main',
            'algoThreeDof = command.three_dof_com:main',
            'algoFourDof = command.four_dof_com:main'
        ],
    },
)
