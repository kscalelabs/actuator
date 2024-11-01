# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import re
import subprocess

from setuptools import find_packages, setup
from setuptools.command.build_ext import build_ext
from setuptools_rust import Binding, RustExtension

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("actuator/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("actuator/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


with open("Cargo.toml", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^version = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in Cargo.toml"
version: str = version_re.group(1)


class RustBuildExt(build_ext):
    def run(self) -> None:
        subprocess.run(["cargo", "run", "--bin", "stub_gen"], check=True)
        super().run()


setup(
    name="actuator",
    version=version,
    description="Python interface for controlling various robotic actuators",
    author="K-Scale Labs",
    url="https://github.com/kscalelabs/actuator",
    rust_extensions=[
        RustExtension(
            target="actuator.bindings",
            path="actuator/bindings/Cargo.toml",
            binding=Binding.PyO3,
        ),
    ],
    setup_requires=["setuptools-rust"],
    zip_safe=False,
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    install_requires=requirements,
    extras_require={"dev": requirements_dev},
    include_package_data=True,
    packages=find_packages(include=["actuator"]),
    cmdclass={"build_ext": RustBuildExt},
)
