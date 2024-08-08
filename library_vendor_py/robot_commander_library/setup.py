from setuptools import setup

setup(name='robot_commander_library',
      version='0.0.1',
      description='Library for interfacing with generative AI models for inference.',
      author='Tomas Horelican',
      license='MIT',
      # Using automatic package discovery
      # packages=["robot_commander_library" "robot_commander_library.ai_interface", "robot_commander_library.commander", "robot_commander_library.utils"],
      classifiers=[
          "Programming Language :: Python :: 3",
          "License :: MIT License"
      ],
      install_requires=[
          'setuptools',
          'dataclasses',
          'typing_extensions',
          'typing',
          # 'subprocess',
          # 'threading',
          # 'os',
          # 'json',
          'pynput',
          # 'random',
          'scipy',
          'sounddevice',
          'soundfile',
          'numpy',
          'nltk',
          'pyaudio',
          'wave',
          'asyncio',
          'requests',
          'pathlib',
          # 'glob'
      ],
      requires=['openai'],
      python_requires='>=3.8',
      include_package_data=True)
