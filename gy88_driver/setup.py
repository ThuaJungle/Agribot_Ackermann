from setuptools import find_packages, setup

package_name = 'gy88_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2t',
    maintainer_email='thuabui1632@gmail.com',
    description='GY-88 Driver for ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tên lệnh = tên_thư_mục.tên_file_code:tên_hàm_chạy
            'gy88_node = gy88_driver.gy88_node:main',
        ],
    },
)