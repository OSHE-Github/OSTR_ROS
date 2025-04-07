from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='oshe',
    maintainer_email='oshe@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                 'hub = py_pubsub.hub:main',
                'receive = py_pubsub.receiver:main',
                'transmit = py_pubsub.transmitter:main',
                'fishhub = py_pubsub.fishHub:main',
                'fishrec = py_pubsub.fishrec:main',
                'fishtran = py_pubsub.fishtran:main',
                'fishspeed = py_pubsub.fishspeed:main',
                'fishturn = py_pubsub.fishturn:main',
                'fishdepth = py_pubsub.fishdepth:main',
                'fishimu = py_pubsub.fishimu:main',
                'fishtemp = py_pubsub.fishtemp:main',
                'fishfake = py_pubsub.fakefishrecv:main',
                'fishbouyancy = py_pubsub.boyancy_contoler:main',
                'fldigirx = py_pubsub.fldigirx:main',
                'fldigitx = py_pubsub.fldigitx:main'

        ],
    },
)
