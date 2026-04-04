from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="robot-traceur-pdf",
    version="0.1.0",
    author="Your Name",
    description="A robot that traces PDF plans on terrain",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/robot-traceur-pdf",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.21.0",
        "matplotlib>=3.5.0",
        "opencv-python>=4.5.0",
        "PyMuPDF>=1.23.0",
        "scipy>=1.7.0",
        "pyserial>=3.5",
        "pyyaml>=6.0",
    ],
)
