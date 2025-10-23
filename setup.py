from setuptools import setup

package_name = 'owl_image_queue'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name, package_name + '.processors'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/orchard_processing.launch.py']),
    ],
    install_requires=['setuptools', 'Pillow'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Image enqueue/processing queue gated by status/velocity topics (owl_sense-friendly)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'enqueue_node = owl_image_queue.enqueue_node:main',
            'processor_node = owl_image_queue.processor_node:main',
            'queue_cli = owl_image_queue.queue_cli:main',
        ],
    },
)
