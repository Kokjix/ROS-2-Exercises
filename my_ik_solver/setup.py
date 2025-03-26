from setuptools import find_packages, setup

package_name = 'my_ik_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_inverse_publisher.py']),  # ✅ Installs launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baran',
    maintainer_email='baranberkbagci@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_inverse_publisher = my_ik_solver.my_inverse_main:main',
            'my_subscriber = my_ik_solver.my_subscriber_example:main',
        ],
    },
)
