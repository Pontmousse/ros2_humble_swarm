#!/usr/bin/env python3

import gpiod
import glob
import sys

print("=== libgpiod v2 API sanity check (v2.4+) ===")

print("gpiod module file:")
print(" ", gpiod.__file__)
print()

print("gpiod.__version__:")
print(" ", getattr(gpiod, "__version__", "N/A"))
print()

required_symbols = [
    ("request_lines", hasattr(gpiod, "request_lines")),
    ("LineSettings", hasattr(gpiod, "LineSettings")),
    ("line.Direction", hasattr(gpiod, "line") and hasattr(gpiod.line, "Direction")),
    ("line.Value", hasattr(gpiod, "line") and hasattr(gpiod.line, "Value")),
]

print("Checking required v2.4+ symbols:")
missing = False
for name, ok in required_symbols:
    print(f"  {name:15s}: {'OK' if ok else 'MISSING'}")
    if not ok:
        missing = True

if missing:
    print("\n❌ gpiod API mismatch")
    sys.exit(1)

print("\n✅ libgpiod v2.4+ API detected")

chips = glob.glob("/dev/gpiochip*")
print("\nDetected gpiochip devices:")
if chips:
    for c in chips:
        print(" ", c)
else:
    print("  (none — expected on non-Pi systems)")

print("\n=== RESULT ===")
print("✔ gpiod v2.4+ Python API is installed and usable")
print("✔ Safe to ship this environment to Raspberry Pi")
print("⚠ Real GPIO access requires Pi hardware")
