#!/usr/bin/python3

import argparse
import json

PUBLIC_MACROS = (
    (
        "RELLUME_PUBLIC_REG", lambda e: "reg" in e and e.get("export"),
        "RELLUME_PUBLIC_REG({name}, {NAME}, {size}, {offset})",
    ),
)

PRIVATE_MACROS = (
    (
        "RELLUME_MAPPED_REG", lambda e: "reg" in e,
        "RELLUME_MAPPED_REG({NAME}, {offset}, LLReg(LL_RT_{reg[0]}, {reg[1]}), Facet::{reg[2]})",
    ),
    (
        "RELLUME_NAMED_REG", lambda e: "name" in e,
        "RELLUME_NAMED_REG({name}, {NAME}, {size}, {offset})",
    ),
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--private", action="store_true")
    parser.add_argument("-o", "--output", type=argparse.FileType("w"), default='-')
    parser.add_argument("description", type=argparse.FileType("r"))
    args = parser.parse_args()

    desc = json.load(args.description)
    off = 0
    for entry in desc:
        if off % entry["size"]:
            raise Exception("misaligned cpu struct entry {}".format(entry))
        entry["offset"] = off
        if "name" in entry:
            entry["NAME"] = entry["name"].upper()
        off += entry["size"]

    macros = PUBLIC_MACROS if not args.private else PRIVATE_MACROS

    res = ""
    for name, include, fmt in macros:
        res += "#ifdef {}\n".format(name)
        for entry in (e for e in desc if include(e)):
            res += fmt.format(**entry) + "\n"
        res += "#endif\n"

    args.output.write(res)
