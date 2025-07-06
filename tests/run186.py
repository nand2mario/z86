#! /usr/bin/env python3

import os
import difflib
import sys
import subprocess

DIR = '80186'
TESTS = ['add', 'bcdcnv', 'bitwise', 'cmpneg', 'control', 'datatrnf', 'shifts', 'rotate', 'sub', 
         'strings', 'mul', 'div', 'interrupt', 
         'jump1', 'jump2', 'rep']

# Notes:
#   div: result are 0xD0 is not checked, as we've changed the INT0 handler

# Skipped tests: 
#   jmpmov
#   segpr: this incorrectly assumes 80386 behavior: div-by-0 pushes IP_THIS instead of IP_AFTER

def hexdump_line(offset, data):
    """Generate a formatted hex dump line for the given data block."""
    hex_bytes = []
    ascii_chars = []
    for byte in data:
        hex_bytes.append(f"{byte:02X}")
        ascii_chars.append(chr(byte) if 32 <= byte <= 126 else '.')
    hex_str = " ".join(hex_bytes)
    ascii_str = "".join(ascii_chars)
    return f"0x{offset:08X}: {hex_str.ljust(47)} |{ascii_str}|"

def generate_hexdump_lines(data):
    """Generate hex dump lines for the entire binary data."""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        lines.append(hexdump_line(i, chunk))
    return lines

# print file1 (actual bits) with annotations from file2 (expected bits)
def compare_files(file1, file2, config={}):
    """Compare two binary files using difflib and print colored unified diff.

    Returns ``True`` if the files are identical and ``False`` otherwise.
    """
    try:
        with open(file1, 'rb') as f1, open(file2, 'rb') as f2:
            data1 = f1.read()
            data2 = f2.read()
    except Exception as e:
        print(f"Error reading files: {e}", file=sys.stderr)
        return False

    # Generate hex dump lines for both files
    lines1 = generate_hexdump_lines(data1)
    lines2 = generate_hexdump_lines(data2)
    
    # Print file information
    print(f"Comparing '{file1}' ({len(data1)} bytes) vs '{file2}' ({len(data2)} bytes)")
    print(f"Total differences: {len([d for d in difflib.ndiff(lines1, lines2) if d[0] != ' '])}")
    print(f"File sizes: {'same' if len(data1) == len(data2) else 'different'}")
    print("-" * 70)
    
    found_differences = False

    # Print colored diff
    for i in range(len(lines1)):
        l1 = lines1[i]
        l2 = lines2[i]
        # compare the bytes: i*16, i*16+1
        line_diff = False
        for j in range(i*16, i*16+16):
            b1 = data1[j]
            b2 = data2[j]
            if j % 2 == 1 and config.get('ignore_of_after') and j >= config['ignore_of_after']:
                b1 = b1 & 0xF7
                b2 = b2 & 0xF7
            if b1 != b2:
                line_diff = True
        # now pretty print the diff
        if line_diff:
            found_differences = True
            for j in range(len(l1)):
                if j >= len(l2) or l1[j] != l2[j]:
                    print(f"\033[91m{l1[j]}\033[0m", end="")
                else:
                    print(l1[j], end="")
            print()
            for j in range(len(l2)):
                if j >= len(l1) or l1[j] != l2[j]:
                    print(f"\033[92m{l2[j]}\033[0m", end="")
                else:
                    print(' ', end="")
            print()

    if not found_differences:
        print("Files are identical")

    return not found_differences


def run_test(test):
    print(f"\nRunning {test}...")

    # special config for some tests
    config = {}
    if test == 'rotate':
        config['ignore_of_after'] = 0x50
        print(f"Ignoring OF after 0x{config['ignore_of_after']:x} for test {test}")

    # get size of result file
    size = os.path.getsize(f"{DIR}/res_{test}.bin")
    if test == 'div':
        size = 0xD0
    os.system(f"obj_dir/Vz86_test {DIR}/{test}.bin results/{test}.mem {size} > results/{test}.txt")
    success = compare_files(f"results/{test}.mem", f"{DIR}/res_{test}.bin", config)
    print("PASSED" if success else "FAILED")
    return success

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    binary_path = os.path.join(script_dir, 'obj_dir', 'Vz86_test')

    if not os.path.isfile(binary_path):
        print("obj_dir/Vz86_test not found, building with make...")
        try:
            subprocess.run(['make'], cwd=script_dir, check=True)
        except subprocess.CalledProcessError:
            print("Error: building Verilator model failed", file=sys.stderr)
            sys.exit(1)
        if not os.path.isfile(binary_path):
            print("Error: obj_dir/Vz86_test still missing after build", file=sys.stderr)
            sys.exit(1)

    os.system('mkdir -p results')
    passed = 0
    total = 1
    if len(sys.argv) > 1:
        if run_test(sys.argv[1]):
            passed += 1
    else:
        total = len(TESTS)
        for test in TESTS:
            if run_test(test):
                passed += 1

    print(f"\nPassed {passed}/{total} tests")
    if passed != total:
        sys.exit(1)
