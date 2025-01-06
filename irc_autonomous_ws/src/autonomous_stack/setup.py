from setuptools import find_packages, setup

package_name = 'autonomous_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subnt',
    maintainer_email='spoorthi262020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input1 = autonomous_stack.inputfirst:main',
            'input2=autonomous_stack.straightpath1:main',
            'controller=autonomous_stack.test_controller_final_with_sp:main',
        ],
    },
)
