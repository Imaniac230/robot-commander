from setuptools import setup

setup(name='robot-commander-library',
      version='0.0.1',
      description='Library for interfacing with generative AI models for inference.',
      author='Tomas Horelican',
      license='MIT',
      packages=["ai_interface", "commander", "utils"],
      classifiers=[
          "Programming Language :: Python :: 3",
          "License :: MIT License"
      ],
      install_requires=[],
      python_requires='>=3.8',
      include_package_data=True)
