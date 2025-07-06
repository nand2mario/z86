# z86 - A High-Performance 80286-Class Softcore CPU

z86 is a pipelined 16-bit x86 CPU softcore that implements real-mode 80286 instructions. It's designed to be a high-performance, compact implementation. It is a work-in-progress, currently capable of booting PC AT BIOS and running IBM BASIC in Verilator simulation.

## Architecture Overview

z86 uses a **5-stage pipeline** to achieve high performance:
- **FETCH**: Instruction fetch with a 32-byte buffer, mostly single-cycle.
- **DECODE1**: Initial instruction decoding and prefix handling.
- **DECODE2**: 2nd stage decoding with effective address calucation and hazard detection.
- **EXECUTE**: ALU operations, memory access, and microcode execution.
- **WRITE-BACK**: Register file updates and flag management.

The pipeline is designed to complete most instructions in a single cycle, with complex instructions handled by microcode for multi-cycle execution. The pipeline design is inspired by the 80486 processor.

## Key Features 

- **5-stage pipeline** implementation
- **8086 compatibility** with all base instructions
- **Real-mode 80286 instruction set** support, e.g. `PUSHA/POPA/INS/OUTS` etc.
- **Hardwired implementation of most instructions** for single-cycle execution.
- **Microcode-driven complex instructions** for multi-cycle operations
- **32-bit wide memory bus** to provide enough bandwidth for instructions and data
- **Comprehensive test suite** with 8088, 80186, and 80286 tests

## 🔄 Work in Progress
- **Cache implementation** (framework exists)
- **Performance optimization** and tuning
- **Additional testing on FPGAs**

## Code Structure
```
src/
├── z86.sv              # Top-level CPU module
├── z86_package.sv      # Package with constants and types
├── fetch.sv            # Instruction fetch and decode-1 stages
├── decode.sv           # Instruction decode-2 stage
├── execute.sv          # Execute stage with ALU and microcode
├── write_back.sv       # Write-back stage
├── regfile.sv          # Register file implementation
├── cache.sv            # Cache module (framework)
├── divider.sv          # Hardware divider
└── ucode.asm           # Microcode assembly
```

## Building and Testing
```bash
# Check out with submodules
git clone --recursive https://github.com/nand2mario/z86.git

# Install Verilator
brew install verilator     # for Mac
sudo apt install verilator # for Linux or WSL

# Build the testbench
cd z86/tests
make

# Run 8086 tests
make test86

# Run 80186 tests
make test186

# Run 80286 tests
make test286
```

## Comparison with ao486

| Feature | z86 | ao486 |
|---------|-----|-------|
| **Lines of Code** | ~6K | ~25K |
| **Pipeline Stages** | 5 | 4 |
| **Mode Support** | Real-mode only | Real/Protected |
| **Cache** | Framework | Implemented |
| **FPU** | No | No |
| **Complexity** | Moderate | High |


## License

This project is licensed under the Apache 2.0 license.

## Acknowledgments

- 8088 test suite: https://github.com/SingleStepTests/8088
- 80186 tests from: http://orbides.1gb.ru/80186_tests.zip
- Inspired by [ao486](https://github.com/alfikpl/ao486.git) softcore
- The i486 CPU: Executing Instructions in One Clock Cycle, John H. Crawford, 1990.

---

*z86 - A high-performance 80286-class softcore CPU for FPGA implementation*
