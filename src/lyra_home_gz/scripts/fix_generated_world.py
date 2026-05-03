#!/usr/bin/env python3

from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: fix_generated_world.py <world_path> <world_name>", file=sys.stderr)
        return 1

    world_path = Path(sys.argv[1])
    world_name = sys.argv[2]

    content = world_path.read_text(encoding="utf-8")
    fixed = content.replace('<world name="world">', f'<world name="{world_name}">', 1)
    world_path.write_text(fixed, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
