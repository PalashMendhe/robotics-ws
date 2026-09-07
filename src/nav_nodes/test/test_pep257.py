# Copyright (c) 2026 Palash Siddharth Mendhe
#
# Licensed under the MIT License. See the LICENSE file in the repository
# root for the full license text.

import os

from ament_pep257.main import main
import pytest

# Lint ONLY this package's sources (see test_flake8.py for rationale).
PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[PKG_DIR])
    assert rc == 0, 'Found code style errors / warnings'
