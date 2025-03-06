from setuptools import find_packages, setup

package_name = 'test05_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='kunama',
    maintainer_email='akana_kim@icloud.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_show_python_test = test05_package.image_show_python_test:main'
        ],
    },
)
