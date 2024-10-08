"""Defines the CLI for the actuator project."""

from actuator.rust.py import hello_world


def main() -> None:
    hello_world()


if __name__ == "__main__":
    # python -m actuator.cli
    main()
