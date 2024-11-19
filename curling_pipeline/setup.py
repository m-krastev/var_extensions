from setuptools import setup
from setuptools.command.install import install

package_name = 'curling_pipeline'

class InstallWithMarker(install):
    def run(self):
        super().run()
        marker_path = f"{self.install_lib}/{package_name}.pth"
        with open(marker_path, "w") as marker_file:
            marker_file.write(f"{self.install_lib}\n")

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    include_package_data=True,
    package_data={
        '': ['package.xml'],
    },
    cmdclass={'install': InstallWithMarker},
    entry_points={
        'console_scripts': [
            f'marker_detection = {package_name}.marker_detection_node:main',
        ],
    },
)