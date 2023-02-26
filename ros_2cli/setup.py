from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_2cli',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    extras_require={
        'completion': ['argcomplete'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/ros_2cli',
        ]),
        ('share/ros2cli', [
            'package.xml',
            'resource/package.dsv',
        ]),
        ('share/ros_2cli/environment', [
            'completion/ROS-argcomplete.bash',
            'completion/ROS-argcomplete.zsh'
        ]),
    ],
    zip_safe=False,
    author='Christophe Bedard',
    author_email='bedard.christophe@gmail.com',
    maintainer='Christophe Bedard',
    maintainer_email='bedard.christophe@gmail.com',
    url='https://github.com/christophebedard/ros-2cli',
    download_url='https://github.com/christophebedard/ros-2cli',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description="Brand-compliant 'ROS 2' and 'ROS 1' command aliases.",
    long_description="""\
This package provides 'ROS 2' and 'ROS 1' command aliases for ROS brand compliance.
The 'ROS 2' command alias supports autocompletion.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ROS = ros_2cli.cli:main',
        ],
    }
)
