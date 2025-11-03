from setuptools import setup, find_packages

package_name = 'owl_image_queue'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}', ['launch/orchard_processing.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'Pillow>=9.0.0',
        'watchdog>=2.1.0',
    ],
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
    python_requires='>=3.8',
)

