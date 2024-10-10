# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import glob
import re

from setuptools import setup
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

package_data = [f"actuator/{name}" for name in ("py.typed", "requirements.txt", "requirements-dev.txt")]
package_data.append("Cargo.toml")
for ext in ("pyi", "rs", "toml", "so"):
    package_data.extend(glob.iglob(f"actuator/**/*.{ext}", recursive=True))


setup(
    name="actuator",
    version=version,
    description="The actuator project",
    author="Benjamin Bolte",
    url="https://github.com/kscalelabs/actuator",
    rust_extensions=[
        RustExtension(
            target="actuator.rust.bindings",
            path="actuator/rust/bindings/Cargo.toml",
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
    package_data={"actuator": package_data},
    packages=["actuator"],
)
