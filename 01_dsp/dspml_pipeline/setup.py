from setuptools import setup, find_packages
from dspml_pipeline import __version__

with open('README.md') as f:
    readme = f.read()

# with open('LICENSE') as f:
#     license = f.read()

setup(
    name='dspml_pipeline',
    version=__version__,
    description='Digital Signal Processing & Machine Learning pipeline for regression of radar data to continuous values.',
    long_description=readme,
    author='Eric Vetha',
    author_email='ericdvet at gmail dot com',
    url='https://github.com/jlab-sensing/wadar',
    # license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)