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

class Assembler:
    def __init__(self, proc):
        self.proc = subprocess.Popen([proc], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
    def assemble(self, code):
        self.proc.stdin.write(code + "\n")
        self.proc.stdin.flush()
        res = self.proc.stdout.readline().strip()
        return bytes.fromhex(res)
    def close(self):
        self.proc.communicate()
        return self.proc.wait()

def parse_case(case, asm=None):
    pre, post = [], []
    cur = pre
    for part in shlex.split(case):
        if part == "=>":
            cur = post
            continue

        key, val = tuple(part.split("=", 2))
        if val == "undef":
            pass
        elif key == "code":
            if cur is not pre:
                raise Exception("code in post-check")
            code = asm.assemble(".intel_syntax noprefix;" + val)
            pre.append("m1000000=" + code.hex() + "cc")
            pre.append("rip=" + struct.pack("<Q", 0x1000000).hex())
            post.append("rip=" + struct.pack("<Q", 0x1000000 + len(code)).hex())
            continue
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
    parser.add_argument("-a", "--assembler")
    parser.add_argument("casefiles", type=argparse.FileType("r"), nargs="+")
    args = parser.parse_args()

    asm = Assembler(args.assembler) if args.assembler else None

    for file in args.casefiles:
        for i, line in enumerate(file.readlines()):
            line = line.strip()
            if not line or line[0] == "#":
                continue

            try:
                case = parse_case(line, asm)
                assert not any(" " in part for part in case)
                args.output.write(" ".join(case) + "\n")
            except Exception as e:
                print("error parsing line", i+1, e)
                raise e

    if asm.close() != 0:
        raise Exception("assembly failed")
