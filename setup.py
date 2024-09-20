# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import re
import subprocess

from setuptools import Command, setup
from setuptools_rust import Binding, RustExtension

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("actuator/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("actuator/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


with open("actuator/__init__.py", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^__version__ = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in actuator/__init__.py"
version: str = version_re.group(1)


class PostInstallCommand(Command):
    """Post-installation for installation mode."""

    description = "Run stub_gen after installation"
    user_options = []

    def initialize_options(self) -> None:
        pass

    def finalize_options(self) -> None:
        pass

    def run(self) -> None:
        subprocess.check_call(["cargo", "run", "--bin", "stub_gen"], cwd="actuator/rust")


setup(
    name="actuator",
    version=version,
    description="The actuator project",
    author="Benjamin Bolte",
    url="https://github.com/kscalelabs/actuator",
    rust_extensions=[
        RustExtension(
            target="actuator.rust.lib",
            path="actuator/rust/Cargo.toml",
            binding=Binding.PyO3,
        )
    ],
    setup_requires=[
        "setuptools-rust",
        "mypy",  # For stubgen
    ],
    include_package_data=True,
    zip_safe=False,
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    install_requires=requirements,
    tests_require=requirements_dev,
    extras_require={"dev": requirements_dev},
    cmdclass={
        "post_install": PostInstallCommand,
    },
)
