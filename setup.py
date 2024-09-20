# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import os
import re
import subprocess

from setuptools import setup
from setuptools.command.build_ext import build_ext as _build_ext
from setuptools_rust import Binding, RustExtension


class BuildExtWithStubgen(_build_ext):
    def run(self) -> None:
        super().run()

        module_name = "actuator.rust.lib"
        output_dir = os.path.dirname(__file__)
        os.makedirs(output_dir, exist_ok=True)
        subprocess.run(["stubgen", "-m", module_name, "-o", output_dir], check=False)


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
    cmdclass={"build_ext": BuildExtWithStubgen},
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
)
