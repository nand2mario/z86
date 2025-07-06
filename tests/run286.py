#!/usr/bin/python3

import os
import sys
import re

TEST=[
    "60.json",
    "61.json",
    "62.json",
    "68.json",
    "69.json",
    "6A.json",
    "6B.json",
    "6C.json",
    "6D.json",
    "6E.json",
    "6F.json",
    "C0.json",
    "C1.json",
]

TEST.sort()

LONG_TEST = [
]

def get_result(result_file):
    tests = 0
    passed = 0
    first_test_case = ""
    try:
        with open(result_file, 'r') as f:
            content = f.read()
            # Find the first test case line
            test_case_match = re.search(r'Running test: (.+?)(?:\n|$)', content)
            if test_case_match:
                first_test_case = test_case_match.group(1)
            
            # Find total passed tests
            match = re.search(r'Total tests: (\d+), passed: (\d+), failed: \d+', content)
            if match:
                tests = int(match.group(1))
                passed = int(match.group(2))
        return (tests, passed, first_test_case)
    except FileNotFoundError:
        pass

if len(sys.argv) > 1 and (sys.argv[1] == "run" or sys.argv[1] == "quick"):
    os.system("rm -rf results_286")
    os.system("mkdir -p results_286")
    if sys.argv[1] == "quick":
        N=200
    else:
        N=10000

    for test in TEST:
        output = test.replace(".json", ".txt")
        long_test = test in LONG_TEST
        long_str = "LONG" if long_test else ""
        long_option = "--long" if long_test else ""
        print(f"Running {test}, output to {output}... {long_str} ", end="")
        os.system(f"obj_dir/Vz86_test --no-trace {long_option} 80286/{test} 0 {N-1} > results_286/{output}")
        (tests, passed, _) = get_result(f"results_286/{output}")
        if tests > 0 and passed / tests < 0.9:
            print('\x1b[31m', end="")      # 32: green, 31: red, 33: yello
        else:
            print('\x1b[32m', end="")
        print(f"{passed}/{tests}", end="")
        print('\x1b[0m')

NAMES = [x.replace(".json", "") for x in TEST]
NAMES.sort()

# Function to create a progress bar
def create_progress_bar(progress, width=50):
    filled = int(width * progress)
    bar = '█' * filled + '░' * (width - filled)
    return f"[{bar}] {progress:.1%}"

# Read results and calculate progress
total_passed = 0
total_tests = 0

print("\nTest Progress:")
print("-" * 100)  # Increased width to accommodate test case names

for name in NAMES:
    result_file = f"results_286/{name}.txt"
    (tests, passed, first_test_case) = get_result(result_file)
    
    progress = passed / tests
    total_passed += passed
    total_tests += tests
    print(f"{name:8} {create_progress_bar(progress)} \t{first_test_case}")

print("\nOverall Progress:")
print("-" * 100)  # Increased width to match above
overall_progress = total_passed / total_tests
print(f"Total: {create_progress_bar(overall_progress)}")
print(f"Passed: {total_passed}/{total_tests} tests")

