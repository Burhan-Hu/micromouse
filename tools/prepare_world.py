"""把 .world 里的 my_maze 模型改为 static，避免被机器人撞跑。

原厂 world 里 my_maze 未设 static，机器人一旦与墙重叠就会把整个迷宫推走，
导致传感器读数与地面真值彻底失配。这里生成一份临时副本，不改动原始文件。

用法: python3 tools/prepare_world.py <src.world> <dst.world> [--raw]
"""
import os
import sys


def main():
    src, dst = sys.argv[1], sys.argv[2]
    raw = "--raw" in sys.argv
    with open(src, encoding="utf-8") as f:
        lines = f.read().split("\n")

    if raw:
        with open(dst, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        print("copied (raw):", dst)
        return 0

    # 原厂 world 里的贴图路径写死成主办方机器（/home/qcmcu/app/custom 等），
    # 本机找不到 -> 地板/墙面渲染成白色。这里改指向本仓库的 custom 目录。
    custom_dir = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "TQD-Micromouse-OC v2.0", "custom")
    out = []
    injected = False
    for line in lines:
        if os.path.isdir(custom_dir) and "<uri>" in line and line.strip().endswith("/custom</uri>"):
            line = line[:line.index("<uri>")] + "<uri>" + custom_dir + "</uri>"
        out.append(line)
        if not injected and "<model name=\"my_maze\">" in line:
            out.append("  <static>true</static>")
            injected = True

    with open(dst, "w", encoding="utf-8") as f:
        f.write("\n".join(out))
    print("prepared:", dst, "| static injected:", injected)
    return 0 if injected else 2


if __name__ == "__main__":
    sys.exit(main())
