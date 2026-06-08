#!/usr/bin/env python3

import subprocess
import re

print("Starting tegrastats...\n")

proc = subprocess.Popen(
    ["tegrastats", "--interval", "1000"],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    universal_newlines=True,
)

try:
    for line in proc.stdout:

        print("\nRAW:")
        print(line.strip())

        # Check POM_5V_IN
        pom_match = re.search(
            r"POM_5V_IN\s+(\d+)(?:mW)?/(\d+)(?:mW)?",
            line
        )

        if pom_match:
            print(
                f"FOUND POM_5V_IN -> current={pom_match.group(1)} mW "
                f"avg={pom_match.group(2)} mW"
            )

        # Check VDD_IN
        vdd_match = re.search(
            r"VDD_IN\s+(\d+)mW/(\d+)mW",
            line
        )

        if vdd_match:
            print(
                f"FOUND VDD_IN -> current={vdd_match.group(1)} mW "
                f"avg={vdd_match.group(2)} mW"
            )

except KeyboardInterrupt:
    proc.kill()
    print("\nStopped.")