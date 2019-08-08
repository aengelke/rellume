#!/usr/bin/python3

import argparse
import shlex
import struct
import subprocess

FMT_SUBST = {
    "b": ("B", lambda v: int(v, 0)),
    "w": ("H", lambda v: int(v, 0)),
    "l": ("L", lambda v: int(v, 0)),
    "q": ("Q", lambda v: int(v, 0)),
}

def parse_case(case):
    pre, post = [], []
    cur = pre
    for part in shlex.split(case):
        if part == "=>":
            cur = post
            continue

        key, val = tuple(part.split("=", 2))
        if val == "undef":
            pass
        elif val[0] == "{":
            raise Exception("not implemented")
        elif ":" in val:
            fmt, nums = tuple(val.split(":", 2))
            fmt, fns = zip(*(FMT_SUBST[c] for c in fmt))
            nums = [fn(v) for fn, v in zip(fns, nums.split(","))]
            val = struct.pack("<" + "".join(fmt), *nums).hex()
        else:
            val = bytes.fromhex(val).hex()
        cur.append("%s=%s"%(key, val))
    return pre + ["=>"] + post

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", type=argparse.FileType("w"), default='-')
    parser.add_argument("casefiles", type=argparse.FileType("r"), nargs="+")
    args = parser.parse_args()

    for file in args.casefiles:
        for line in file.readlines():
            line = line.strip()
            if not line or line[0] == "#":
                continue

            case = parse_case(line)
            assert not any(" " in part for part in case)
            args.output.write(" ".join(case) + "\n")
