from setuptools import setup

package_name = 'llm_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='zeus',
    maintainer_email='zeus@todo.todo',
    description='LLM-based teleop + YOLO scene description via Ollama',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_teleop = llm_teleop.llm_node:main',
            'yolo_scene = llm_teleop.yolo_scene_node:main',
            'explain_scene_cli = llm_teleop.explain_scene_cli:main',
        ],
    },
)
