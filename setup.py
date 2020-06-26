from setuptools import setup, find_packages

setup(name='cairo_motion_planning',
      version='0.1',
      description='Package containing interfaces and algorithms for motion planning.',
      url='https://github.com/cairo-robotics/cairo_motion_planning',
      author='Carl Mueller',
      author_email='carl.mueller@colorado.edu',
      license='',
      packages=find_packages(),
      install_requires=["scikit-learn==0.23.1", "python-igraph==0.8.2"])