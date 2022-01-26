#!/usr/bin/python3

import argparse
import shlex
import struct
import subprocess

FMT_SUBST = {
    "b": ("B", lambda v: int(v, 0) % 0x100),
    "w": ("H", lambda v: int(v, 0) % 0x10000),
    "l": ("L", lambda v: int(v, 0) % 0x100000000),
    "q": ("Q", lambda v: int(v, 0) % 0x10000000000000000),
    "f": ("f", lambda v: float(v)),
}

class Assembler:
    def __init__(self, proc, arch):
        self.proc = subprocess.Popen([proc, arch], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
        self.arch = arch
    def assemble(self, code):
        self.proc.stdin.write("!ASM " + code + "\n")
        self.proc.stdin.flush()
        res = bytes.fromhex(self.proc.stdout.readline().strip())
        if self.arch == "x86_64":
            return res, b"\xcc" # term: int 3
        if self.arch == "rv64":
            return res, b"\x73\x00\x10\x00"
        if self.arch == "aarch64":
            return res, b"\x00\x00\x40\xd4" # term: hlt #0
    def close(self):
        self.proc.communicate()
        return self.proc.wait()

def parse_case(case, asm=None):
    pre, post = [], []
    cur = pre
    for i, part in enumerate(shlex.split(case)):
        if part == "!" and i == 0:
            pre.append("!")
            continue
        if part[:1] in "+-" and cur is pre:
            pre.append(part)
            continue
        if part == "=>":
            cur = post
            continue

        key, val = tuple(part.split("=", 2))
        if val == "undef":
            pass
        elif key == "code":
            if cur is not pre:
                raise Exception("code in post-check")
            code, term = asm.assemble(val)
            pre.append("m1000000=" + code.hex() + term.hex())
            ripstr = "rip=" if asm.arch == "x86_64" or asm.arch == "rv64" else "pc="
            pre.append(ripstr + struct.pack("<Q", 0x1000000).hex())
            post.append(ripstr + struct.pack("<Q", 0x1000000 + len(code)).hex())
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
    parser.add_argument("-A", "--arch")
    parser.add_argument("casefiles", type=argparse.FileType("r"), nargs="+")
    args = parser.parse_args()

    asm = Assembler(args.assembler, args.arch) if args.assembler else None

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
