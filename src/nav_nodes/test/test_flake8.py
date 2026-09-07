# Copyright (c) 2026 Palash Siddharth Mendhe
#
# Licensed under the MIT License. See the LICENSE file in the repository
# root for the full license text.

import os

from ament_flake8.main import main_with_errors
import pytest

# Lint ONLY this package's sources. The unscoped default ('.') lints
# whatever the invocation cwd is — the whole workspace when run via
# `pytest` from the repo root — which fails on other packages' style and
# makes this test nondeterministic. Scoping matches `colcon test` behavior.
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[PKG_DIR])
    assert rc == 0, (
        f'Found {len(errors)} code style errors / warnings:\n'
        + '\n'.join(errors))
