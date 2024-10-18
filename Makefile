# Makefile

# ------------------------ #
#          Build           #
# ------------------------ #

install:
	cargo run --bin stub_gen
	@touch setup.py
	@uv pip install -e '.[dev]'
.PHONY: build

# ------------------------ #
#       Static Checks      #
# ------------------------ #

py-files := $(shell find . -name '*.py')

format:
	@black $(py-files)
	@ruff format $(py-files)
	@cargo fmt
.PHONY: format

format-cpp:
	@clang-format -i $(shell find . -name '*.cpp' -o -name '*.h')
	@cmake-format -i $(shell find . -name 'CMakeLists.txt' -o -name '*.cmake')
.PHONY: format-cpp

static-checks:
	@black --diff --check $(py-files)
	@ruff check $(py-files)
	@mypy --install-types --non-interactive $(py-files)
.PHONY: lint

mypy-daemon:
	@dmypy run -- $(py-files)
.PHONY: mypy-daemon

# ------------------------ #
#        Unit tests        #
# ------------------------ #

test:
	python -m pytest
.PHONY: test
