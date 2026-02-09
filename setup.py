from setuptools import setup, find_packages
import os

package_name = 'muro_bridge'


def package_files(directory):
    """
    Recursively collect all files under a directory
    and preserve the directory structure in install space.
    """
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join(
                'share', package_name, path
            )
            paths.append((install_path, [file_path]))
    return paths


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(where='src'),
    package_dir={'': 'src'},

    data_files=[
        # ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml'],
        ),

        # launch files (preserve structure)
        *package_files('launch'),

        # ✅ config files (preserve full folder structure)
        *package_files('config'),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='allen',
    maintainer_email='allenbecareful@gmail.com',
    description='MURO multi-robot bridge and relay tools',
    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'relay = muro_bridge.relay:main',
            'tf_aggregator = muro_bridge.tf_aggregator:main',
            'multi_bridge = muro_bridge.multi_bridge:main',
        ],
    },
)





