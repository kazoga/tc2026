"""symlink installでもros2 run/launchから補助ノードを起動できることを確認する."""

import os
from pathlib import Path


def test_cmake_programs_have_executable_sources() -> None:
    root = Path(__file__).parents[1]
    block = (root/'CMakeLists.txt').read_text().split('  PROGRAMS\n', 1)[1]
    programs = block.split('  DESTINATION', 1)[0].split()
    missing = [name for name in programs if not os.access(root/name, os.X_OK)]
    assert not missing, f'symlink installで実行できません: {missing}'
