#!/usr/bin/python3

import os
import sys
import re

TEST=[  "00.json", "32.json", "82.2.json", "A6.json", "D0.5.json", "E8.json",
        "01.json", "33.json", "82.3.json", "A7.json", "E9.json",
        "02.json", "34.json", "82.4.json", "A8.json", "D0.7.json", "EA.json",
        "03.json", "35.json", "82.5.json", "A9.json", "D1.0.json", "EB.json",
        "04.json", "37.json", "82.6.json", "AA.json", "D1.1.json", "EC.json",
        "05.json", "38.json", "82.7.json", "AB.json", "D1.2.json", "ED.json",
        "06.json", "39.json", "83.0.json", "AC.json", "D1.3.json", "EE.json",
        "07.json", "3A.json", "83.1.json", "AD.json", "D1.4.json", "EF.json",
        "08.json", "3B.json", "83.2.json", "AE.json", "D1.5.json", "F5.json",
        "09.json", "3C.json", "83.3.json", "AF.json", "F6.0.json",
        "0A.json", "3D.json", "83.4.json", "B0.json", "D1.7.json", "F6.1.json",
        "0B.json", "3F.json", "83.5.json", "B1.json", "D2.0.json", "F6.2.json",
        "0C.json", "40.json", "83.6.json", "B2.json", "D2.1.json", "F6.3.json",
        "0D.json", "41.json", "70.json", "83.7.json", "B3.json", "D2.2.json", "F6.4.json",
        "0E.json", "42.json", "71.json", "84.json", "B4.json", "D2.3.json", "F6.5.json",
        "10.json", "43.json", "72.json", "85.json", "B5.json", "D2.4.json", "F6.6.json",
        "11.json", "44.json", "73.json", "86.json", "B6.json", "D2.5.json", "F6.7.json",
        "12.json", "45.json", "74.json", "87.json", "B7.json", "F7.0.json",
        "13.json", "46.json", "75.json", "88.json", "B8.json", "D2.7.json", "F7.1.json",
        "14.json", "47.json", "76.json", "89.json", "B9.json", "D3.0.json", "F7.2.json",
        "15.json", "48.json", "77.json", "8A.json", "BA.json", "D3.1.json", "F7.3.json",
        "16.json", "49.json", "78.json", "8B.json", "BB.json", "D3.2.json", "F7.4.json",
        "17.json", "4A.json", "79.json", "8C.json", "BC.json", "D3.3.json", "F7.5.json",
        "18.json", "4B.json", "7A.json", "8D.json", "BD.json", "D3.4.json", "F7.6.json",
        "19.json", "4C.json", "7B.json", "8E.json", "BE.json", "D3.5.json", "F7.7.json",
        "1A.json", "4D.json", "7C.json", "8F.json", "BF.json", "F8.json",
        "1B.json", "4E.json", "7D.json", "90.json", "C0.json", "D3.7.json", "F9.json",
        "1C.json", "4F.json", "7E.json", "91.json", "C1.json", "D4.json", "FA.json",
        "1D.json", "50.json", "7F.json", "92.json", "C2.json", "D5.json", "FB.json",
        "1E.json", "51.json", "80.0.json", "93.json", "C3.json", "FC.json",
        "1F.json", "52.json", "80.1.json", "94.json", "C4.json", "D7.json", "FD.json",
        "20.json", "53.json", "80.2.json", "95.json", "C5.json", "D8.json", "FE.0.json",
        "21.json", "54.json", "80.3.json", "96.json", "C6.json", "D9.json", "FE.1.json",
        "22.json", "55.json", "80.4.json", "97.json", "C7.json", "DA.json", "FF.0.json",
        "23.json", "56.json", "80.5.json", "98.json", "DB.json", "FF.1.json",
        "24.json", "57.json", "80.6.json", "99.json", "DC.json", "FF.2.json",
        "25.json", "58.json", "80.7.json", "9A.json", "CA.json", "DD.json", "FF.3.json",
        "27.json", "59.json", "81.0.json", "9C.json", "CB.json", "DE.json", "FF.4.json",
        "28.json", "5A.json", "81.1.json", "9D.json", "CC.json", "DF.json", "FF.5.json",
        "29.json", "5B.json", "81.2.json", "9E.json", "CD.json", "E0.json", "FF.6.json",
        "2A.json", "5C.json", "81.3.json", "9F.json", "CE.json", "E1.json", "FF.7.json",
        "2B.json", "5D.json", "81.4.json", "A0.json", "CF.json", "E2.json", 
        "2C.json", "5E.json", "81.5.json", "A1.json", "D0.0.json", "E3.json",
        "2D.json", "5F.json", "81.6.json", "A2.json", "D0.1.json", "E4.json",
        "2F.json", "81.7.json", "A3.json", "D0.2.json", "E5.json",
        "30.json", "82.0.json", "A4.json", "D0.3.json", "E6.json",
        "31.json", "82.1.json", "A5.json", "D0.4.json", "E7.json"]

TEST.sort()

LONG_TEST = [
    "A4.json", "A5.json", "A6.json", "A7.json", "AA.json", "AB.json", "AC.json", "AD.json", "AE.json", "AF.json"
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
    os.system("rm -rf results")
    os.system("mkdir -p results")
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
        os.system(f"obj_dir/Vz86_test --no-trace {long_option} 8088/{test} 0 {N-1} > results/{output}")
        (tests, passed, _) = get_result(f"results/{output}")
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
    result_file = f"results/{name}.txt"
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

