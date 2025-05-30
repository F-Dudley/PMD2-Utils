from setuptools import setup, find_namespace_packages


def read_requirements(file_path):
    """Read the requirements from a file and return them as a list."""
    with open(file_path, "r") as file:
        requirements = file.readlines()
    return [req.strip() for req in requirements if req.strip()]


setup(
    name="PMD_Utils",
    version="0.1.0",
    author="F-Dudley",
    description="Utilities for interacting with ElmorLabs Power Monitoring Devices (PMD)",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    packages=find_namespace_packages(include=["PMD_Utils.*"]),
    package_data={
        "PMD_Utils": [
            "README.md",
            "requirements.txt",
        ]
    },
    requires=read_requirements("requirements.txt"),
    install_requires=read_requirements("requirements.txt"),
)
