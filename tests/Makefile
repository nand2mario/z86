VERILATOR = verilator
VERILATOR_FLAGS = +1800-2017ext+sv --trace-fst --trace-structs --top-module z86_test --cc --exe --build -CFLAGS "-I../jsoncons/include -std=c++17" -j 0 -Wno-WIDTH -Wno-PINMISSING
VERILATOR_INCLUDE = -I../src 
VERILATOR_OPT = -O2

# Source files
SOURCES = ../src/z86_package.sv ../src/z86.sv ../src/cache.sv ../src/regfile.sv ../src/fetch.sv ../src/decode.sv \
          ../src/execute.sv ../src/write_back.sv ../src/divider.sv sdram_sim.sv z86_test.sv
DEPS = ../src/ucode_entry.svh  ../src/ucode_rom.svh ../src/fetch_lut.svh
CPP_SOURCES = main.cpp

# Default target
all: obj_dir/Vz86_test

../src/fetch_lut.svh: ../tools/fetch_lut.py
	cd ../src && python3 ../tools/fetch_lut.py

../src/ucode_rom.svh ../src/ucode_entry.svh: ../src/ucode.asm
	cd ../src && python3 ../tools/ucode.py ucode.asm

# Generate Verilator files and build
obj_dir/Vz86_test: $(SOURCES) $(CPP_SOURCES) $(DEPS)
	$(VERILATOR) $(VERILATOR_FLAGS) $(VERILATOR_INCLUDE) $(VERILATOR_OPT) $(SOURCES) $(CPP_SOURCES) 

# Run Verilator simulation
run_verilator: obj_dir/Vz86_test
	obj_dir/Vz86_test 8088/00.json 0

test86: obj_dir/Vz86_test
	./run86.py quick

test186: obj_dir/Vz86_test
	./run186.py

test286: obj_dir/Vz86_test
	./run286.py quick

test: test86 test186 test286

# Clean generated files
clean:
	rm -rf obj_dir
	rm -f *.o *.d 

.PHONY: all microcode run_verilator test clean
