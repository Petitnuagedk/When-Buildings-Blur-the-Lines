#!/usr/bin/env python3
"""
A simple terminal "aesthetic" launcher script that prints

  - a large green `NAB\u00dbSTER` title
  - a large blue `VID\u00c9DOGRAMME` subtitle
  - a little `corp.` inside a square box
  - then continues to spew fake log/loading information

It uses `rich` for colours/boxes and `pyfiglet` to generate the
ASCII-art fonts.  Install dependencies with

    pip install rich pyfiglet

and run from a POSIX terminal (Linux/macOS) with

    ./fancy_logo.py

Feel free to tweak the fonts, timing and log format for your own aesthetic.
"""

import time
import random
from datetime import datetime

from rich.console import Console
from rich.panel import Panel
from rich import box
from pyfiglet import Figlet

console = Console()

# create the ascii art using pyfiglet
fig = Figlet(font="standard")
nabuster = fig.renderText("NABÛSTER")
vide = fig.renderText("VIDÉDOGRAMME")

# print the header with colours
console.print(f"[green]{nabuster}[/green]")
console.print(f"[blue]{vide}[/blue]")

# little corporation tag in a square
corp_panel = Panel("corp.", box=box.SQUARE, width=len("corp.") + 4, style="white")
console.print(corp_panel)

# generate fake log entries forever (Ctrl‑C to stop)
try:
    i = 0
    # messages inspired by Windows blue screen style errors
    fragments = [
        "IRQL_NOT_LESS_OR_EQUAL",
        "PAGE_FAULT_IN_NONPAGED_AREA",
        "KERNEL_DATA_INPAGE_ERROR",
        "SYSTEM_THREAD_EXCEPTION_NOT_HANDLED",
        "BAD_POOL_HEADER",
        "UNEXPECTED_KERNEL_MODE_TRAP",
        "NTFS_FILE_SYSTEM",
        "DRIVER_POWER_STATE_FAILURE",
        "MACHINE_CHECK_EXCEPTION",
    ]

    def random_hex(length=8):
        return ''.join(random.choice('0123456789ABCDEF') for _ in range(length))

    while True:
        i += 1
        # occasionally spit out a long warning paragraph
        if random.random() < 0.25:
            warning = (
                "[red bold]WARNING:[/] "
                "If this is the first time you see this error, stop the computer immediately. "
                "Do not attempt to reboot or modify system files. "
                "Contact the administrator and quote code 0x" + random_hex(6) + ". "
                "Further actions may result in data loss or void warranty."
            )
            console.print(Panel(warning, style="red", expand=True))
            # small pause so it stands out
            time.sleep(0.2)
            continue

        level = random.choice(["INFO", "DEBUG", "WARN", "ERROR", "TRACE", "FATAL"])
        ts = datetime.now().strftime("%H:%M:%S")
        frag = random.choice(fragments)
        hexval = random_hex(12)
        # take up more horizontal space by padding
        msg = f"{ts} [{level}] component={i:03d} | {frag} | 0x{hexval}"
        # if console width known, pad out to near width with dots
        width = console.size.width
        if width and len(msg) < width - 4:
            msg = msg.ljust(width - 4, '.')
        console.print(msg, style="dim")
        time.sleep(0.05)
except KeyboardInterrupt:
    console.print("\n[bold green]Shutdown requested, exiting.[/]")
