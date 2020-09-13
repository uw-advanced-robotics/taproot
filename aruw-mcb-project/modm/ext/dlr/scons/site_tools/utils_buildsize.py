#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013, 2016-2017, German Aerospace Center (DLR)
# Copyright (c) 2017-2018, Niklas Hauser
# Copyright (c) 2018, Fabian Greif
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Authors:
# - 2013, 2016-2017, Fabian Greif (DLR RY-AVS)
# - 2017-2018, Niklas Hauser
# - 2018, Fabian Greif

# USAGE:
#
# Reads and matches ELF sections and segments to the physical memories
# passed in CONFIG_DEVICE_MEMORY which needs to be a list of dictionaries:
#
#     env["CONFIG_DEVICE_MEMORY"] = [
#         {"name": "flash", "start": 0x08000000, "size": 2097152, "access": "rx"},
#         {"name": "ccm", "start": 0x10000000, "size": 65536, "access": "rw"},
#         {"name": "sram1", "start": 0x20000000, "size": 163840, "access": "rwx"}
#     ]

import textwrap

from SCons.Script import *
from collections import defaultdict

try:
    from elftools.elf.elffile import ELFFile
except:
    print("elftools are missing, you need to `pip install pyelftools`!")
    exit(1)


def human_readable_format(num, unit='B'):
    for prefix in ['', 'Ki', 'Mi', 'Gi', 'Ti']:
        if abs(num) < 1024.0:
            if prefix == '':
                # Align the output independent of whether a prefix is used
                return "%3.1f %s  " % (num, unit)
            else:
                return "%3.1f %s%s" % (num, prefix, unit)
        num /= 1024.0
    return "%.1f %s%s" % (num, 'Pi', unit)


def size_action(target, source, env):
    memories = defaultdict(list)
    for memory in env["CONFIG_DEVICE_MEMORY"]:
        if "w" in memory["access"]:
            memories["ram"].append(memory)
        else:
            memories["rom"].append(memory)

    memory_sections = []
    with open(source[0].path, "rb") as src:
        elffile = ELFFile(src)
        for section in elffile.iter_sections():
            s = {
                "name":  section.name,
                "vaddr": section["sh_addr"],
                "paddr": section["sh_addr"],
                "size": section["sh_size"],
            }
            # if using python 2 there is a need for .decode
            if (sys.version_info[0] == 2):
                s["name"] = s["name"].decode('ASCII')
            if s["vaddr"] == 0 or s["size"] == 0: continue;
            for segment in elffile.iter_segments():
                if (segment["p_vaddr"] == s["vaddr"] and segment["p_filesz"] == s["size"]):
                    s["paddr"] = segment["p_paddr"]
                    break
            memory_sections.append(s)

    sections = defaultdict(list)
    totals = defaultdict(int)
    for s in memory_sections:
        if s["name"].startswith(".stack"):
            totals["stack"] += s["size"]
            sections["stack"].append(s["name"])
        elif s["name"].startswith(".heap"):
            totals["heap"] += s["size"]
            sections["heap"].append(s["name"])
        else:

            def is_in_memory(name):
                start = s[{"rom": "paddr", "ram": "vaddr"}[name]]
                return any(((m["start"] <= start) and
                            ((start + s["size"]) <= (m["start"] + m["size"])))
                            for m in memories[name])

            if is_in_memory("rom"):
                totals["rom"] += s["size"]
                sections["rom"].append(s["name"])
            if is_in_memory("ram"):
                totals["static"] += s["size"]
                sections["static"].append(s["name"])

    # create lists of the used sections for Flash and RAM
    sections["rom"] = sorted(sections["rom"])
    sections["ram"] = sorted(list(set(sections["static"] + sections["stack"])))
    sections["heap"] = sorted(sections["heap"])

    flash = sum(m["size"] for m in memories["rom"])
    ram = sum(m["size"] for m in memories["ram"])

    subs = {
        "ram": totals["static"] + totals["stack"],
        "rom_s": "\n ".join(textwrap.wrap(" + ".join(sections["rom"]), 80)),
        "ram_s": "\n ".join(textwrap.wrap(" + ".join(sections["ram"]), 80)),
        "heap_s": "\n ".join(textwrap.wrap(" + ".join(sections["heap"]), 80)),
        "rom_p": totals["rom"] / float(flash) * 100.0,
        "ram_p": (totals["static"] + totals["stack"]) / float(ram) * 100.0,
        "static_p": totals["static"] / float(ram) * 100.0,
        "stack_p": totals["stack"] / float(ram) * 100.0,
        "heap_p": totals["heap"] / float(ram) * 100.0
    }
    subs.update(totals)

    print("""
Program: {rom_fmt:>9s} ({rom_p:2.1f}% used)
({rom_s})

Data:    {ram_fmt:>9s} ({ram_p:2.1f}% used) = {static} B static ({static_p:2.1f}%) + {stack} B stack ({stack_p:2.1f}%)
({ram_s})

Heap:  {heap_fmt:>11s} ({heap_p:2.1f}% available)
({heap_s})
""".format(ram_fmt=human_readable_format(subs["ram"]),
           rom_fmt=human_readable_format(subs["rom"]),
           heap_fmt=human_readable_format(subs["heap"]),
           **subs))


def show_size(env, source, alias='__size'):
    if env.has_key('CONFIG_DEVICE_MEMORY'):
        action = Action(size_action, cmdstr="$SIZECOMSTR")
    elif env.has_key('CONFIG_DEVICE_NAME'):
        action = Action("$SIZE -C --mcu=$CONFIG_DEVICE_NAME %s" % source[0].path,
                        cmdstr="$SIZECOMSTR")
    else:
        # use the raw output of the size tool
        action = Action("$SIZE %s" % source[0].path,
                        cmdstr="$SIZECOMSTR")

    return env.AlwaysBuild(env.Alias(alias, source, action))


def generate(env, **kw):
    env.AddMethod(show_size, 'Size')


def exists(env):
    return True
