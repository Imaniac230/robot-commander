from pathlib import Path
from setuptools import setup

# with open(Path(__file__).resolve().parent / 'README.md', encoding='utf-8') as f: long_description = f.read()

setup(name='robot-commander',
      version='0.0.1',
      description='Bridging generative AI with ROS2 for smart robot agents.',
      author='Tomas Horelican',
      license='MIT',
      # long_description=long_description,
      # long_description_content_type='text/markdown',
      packages = ["ai_interface", "commander", "utils"],
      classifiers=[
        "Programming Language :: Python :: 3",
        "License :: MIT License"
      ],
      install_requires=[],
      python_requires='>=3.8',
      include_package_data=True)
