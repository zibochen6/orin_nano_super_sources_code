from setuptools import find_packages, setup

package_name = 'learning_dds'

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
    maintainer='seeed',
    maintainer_email='www.1691125058@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        # 发布者节点：命令名 = 包名.文件名:main函数
        'dds_controller_pub = learning_dds.dds_controller_pub:main',
        # 订阅者节点
        'dds_robot_sub = learning_dds.dds_robot_sub:main',
    ],
},
)
