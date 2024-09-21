"""Defines the CLI for the actuator project."""

from actuator.rust.lib import hello_world, mean


def main() -> None:
    hello_world()
    print(mean([1, 2, 3, 4, 5]))


if __name__ == "__main__":
    # python -m actuator.cli
    main()
