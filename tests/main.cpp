#include "Vz86_test.h"
#include "verilated.h"
#include "verilated_fst_c.h"
#include <jsoncons/json.hpp>
#include <fstream>
#include <iostream>
#include <set>

#include "Vz86_test_z86_test.h"
#include "Vz86_test_z86.h"
#include "Vz86_test_execute.h"
#include "Vz86_test_regfile.h"
#include "Vz86_test_z86_package.h"
#include "Vz86_test_write_back.h"
#include "Vz86_test_sdram_sim.h"

// z86 test bench
// 8088 test suite: https://github.com/SingleStepTests/8088
// 80186 tests:     http://orbides.1gb.ru/80186_tests.zip

bool trace_on = true;
// bool long_test = false;
int test_length = 200;
uint64_t sim_time = 0;
Vz86_test tb;
VerilatedFstC* trace;
int failure = -1;
uint16_t ignore_mask = 0xf400;      // 15:12 
int ignore_memory = 0;

struct Regs {
    jsoncons::optional<uint16_t> ax, bx, cx, dx, cs, ss, ds, es, sp, bp, si, di, ip, flags;
};

void regs2array(Regs &regs, uint16_t *array) {
    array[0] = regs.ax.value_or(array[0]);
    array[1] = regs.cx.value_or(array[1]);
    array[2] = regs.dx.value_or(array[2]);
    array[3] = regs.bx.value_or(array[3]);
    array[4] = regs.sp.value_or(array[4]);
    array[5] = regs.bp.value_or(array[5]);
    array[6] = regs.si.value_or(array[6]);
    array[7] = regs.di.value_or(array[7]);
    array[8] = regs.cs.value_or(array[8]);
    array[9] = regs.ss.value_or(array[9]);
    array[10] = regs.ds.value_or(array[10]);
    array[11] = regs.es.value_or(array[11]);
    array[12] = regs.ip.value_or(array[12]);
    array[13] = regs.flags.value_or(array[13]);
    // printf("SS = %04x\n", array[9]);
}

struct InitialFinal {
    Regs regs;
    std::vector<std::pair<uint32_t, uint8_t>> ram;  // [address, value]
    std::vector<uint8_t> queue;
};

struct TestCase {
    int idx;
    std::string name;
    std::vector<uint8_t> bytes;
    InitialFinal initial;
    InitialFinal final;

    uint16_t regs[14];
    uint16_t regs_expected[14];
};

JSONCONS_N_MEMBER_TRAITS(
    Regs, 0, 
    ax, bx, cx, dx, cs, ss, ds, es, sp, bp, si, di, ip, flags
)

JSONCONS_ALL_MEMBER_TRAITS(
    InitialFinal,
    regs, ram, queue
)

JSONCONS_ALL_MEMBER_TRAITS(
    TestCase,
    idx, name, bytes, initial, final
)

void step() {
    tb.clk_sys = !tb.clk_sys;
    tb.eval();
    sim_time++;
    if (trace_on) {
        trace->dump(sim_time);
    }
}

void reset() {
    tb.cpu_reset = 1;
    step();
    step();
    tb.cpu_reset = 0;
    step();
}

void load_program(std::vector<uint8_t> &program) {
    if (tb.clk_sys) step();    // make sure clk=0
    
    for (int i = 0; i < program.size(); i++) {
        tb.dbg_mem_wr = 1;
        tb.dbg_mem_addr = 0x0000 + i;
        tb.dbg_mem_din = program[i];
        step(); step();
    }
    tb.dbg_mem_wr = 0;
    step(); step();
}

void set_regs(uint16_t *regs) {
    if (tb.clk_sys) step();    // make sure clk=0

    for (int i = 0; i < 14; i++) {
        tb.dbg_reg_wr = 1;
        tb.dbg_reg_addr = i;
        tb.dbg_reg_din = regs[i];
        step(); step();
        tb.dbg_reg_wr = 0;
        step(); step();
    }
}

void set_ram(std::vector<std::pair<uint32_t, uint8_t>> &ram) {
    std::set<uint32_t> addresses;

    if (tb.clk_sys) step();    // make sure clk=0
    for (int i = 0; i < ram.size(); i++) {
        tb.dbg_mem_wr = 1;
        tb.dbg_mem_addr = ram[i].first;
        tb.dbg_mem_din = ram[i].second;
        step(); step();

        addresses.insert(ram[i].first);
    }

    // Add extra HLT (F4h) instructions, so we know when to stop the simulation
    for (std::set<uint32_t>::iterator it = addresses.begin(); it != addresses.end(); it++) {
        uint32_t a = *it;
        if (std::next(it) == addresses.end() || *std::next(it) != a+1) {  // non-consecutive address, add a HLT after it
            tb.dbg_mem_wr = 1;
            tb.dbg_mem_addr = a+1;
            tb.dbg_mem_din = 0xF4;
            step(); step();
        }
    }

    tb.dbg_mem_wr = 0;
    step(); step();
}

void run_program(int cycles, bool finish_on_hlt) {
    uint64_t start_time = sim_time;
    Vz86_test_execute *execute = tb.z86_test->z86->u_execute;
    int halt_at = cycles;
    printf("Running program for %d cycles\n", cycles);
    for (int i = 0; i < cycles && i < halt_at; i++) {
        if (finish_on_hlt && halt_at == cycles && execute->halted) {
            printf("Executing HLT - test is done. Finish simulation.\n");
            halt_at = i+10;   // allow 5 more cycles to finish memory operations amd etc

            uint64_t end_time = sim_time;
            printf("CPI: %.3f\n", (end_time - start_time) / (double)execute->retire_counter / 2);  // every clock is 2 sim_time steps
        }
        step(); step();
    }
}

std::ostream& operator<<(std::ostream& os, const Regs &regs) {
    os << std::hex;
    if (regs.ax.has_value()) os << "AX=" << regs.ax.value() << " ";
    if (regs.bx.has_value()) os << "BX=" << regs.bx.value() << " ";
    if (regs.cx.has_value()) os << "CX=" << regs.cx.value() << " ";
    if (regs.dx.has_value()) os << "DX=" << regs.dx.value() << " ";
    if (regs.sp.has_value()) os << "SP=" << regs.sp.value() << " ";
    if (regs.bp.has_value()) os << "BP=" << regs.bp.value() << " ";
    if (regs.si.has_value()) os << "SI=" << regs.si.value() << " ";
    if (regs.di.has_value()) os << "DI=" << regs.di.value() << " ";
    if (regs.cs.has_value()) os << "CS=" << regs.cs.value() << " ";
    if (regs.ss.has_value()) os << "SS=" << regs.ss.value() << " ";
    if (regs.ds.has_value()) os << "DS=" << regs.ds.value() << " ";
    if (regs.es.has_value()) os << "ES=" << regs.es.value() << " ";
    if (regs.ip.has_value()) os << "IP=" << regs.ip.value() << " ";
    if (regs.flags.has_value()) os << "FLAGS=" << regs.flags.value();
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<uint8_t> &v) {
    for (int i = 0; i < v.size(); i++) {
        os << std::hex << (int)v[i] << " ";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<std::pair<uint32_t, uint8_t>> &v) {
    for (int i = 0; i < v.size(); i++) {
        os << std::hex << v[i].first << ":" << (int)v[i].second << " ";
    }
    return os;
}

bool run_test(TestCase &t) {
    printf("Running test: %s\n", t.name.c_str());

    regs2array(t.initial.regs, t.regs);
    regs2array(t.initial.regs, t.regs_expected);
    regs2array(t.final.regs, t.regs_expected);

    tb.reset = 1;
    tb.cpu_reset = 1;
    step(); step(); step(); step();
    tb.reset = 0;
    step(); step();
    
    set_ram(t.initial.ram);
    set_regs(t.regs);               // this also sets CS:IP and fetching address
    
    tb.cpu_reset = 0;
    step(); step();
    
    run_program(test_length, true);

    tb.cpu_reset = 1;
    step(); step(); step(); step();
    
    bool success = true;

    // check register values
    for (int i = 0; i < 14; i++) {
        if (i == 12) continue;      // skip IP check
    #if 1
        if (i < 8)
            t.regs[i] = tb.z86_test->z86->u_regfile->regs[i];
        else switch (i) {
            case 8: 
                t.regs[i] = tb.z86_test->z86->u_write_back->seg_CS;
                break;
            case 9: 
                t.regs[i] = tb.z86_test->z86->u_write_back->seg_SS;
                break;
            case 10:
                t.regs[i] = tb.z86_test->z86->u_write_back->seg_DS;
                break;
            case 11:
                t.regs[i] = tb.z86_test->z86->u_write_back->seg_ES;
                break;
            case 12:
                t.regs[i] = tb.z86_test->z86->u_write_back->reg_ip;
                break;
            case 13:
                t.regs[i] = tb.z86_test->z86->u_execute->reg_f;
                break;
        }
    #else
        tb.dbg_reg_rd = 1;
        tb.dbg_reg_addr = i;
        step(); step();
        t.regs[i] = tb.dbg_reg_dout;
    #endif

        if (i == 13) {
            t.regs[i] = t.regs[i] & ~ignore_mask;
            t.regs[i] |= (t.regs_expected[i] & ignore_mask);
        }

        if (t.regs[i] != t.regs_expected[i]) {
            printf("Register %d MISMATCH: %04x != %04x\n", i, t.regs[i], t.regs_expected[i]);
            if (i == 13) {
                printf("  Expected flags: CF=%d PF=%d AF=%d ZF=%d SF=%d TF=%d IF=%d, DF=%d OF=%d\n", 
                    t.regs_expected[13] & 1, t.regs_expected[13] >> 2 & 1, t.regs_expected[13] >> 4 & 1, t.regs_expected[13] >> 6 & 1, 
                    t.regs_expected[13] >> 7 & 1, t.regs_expected[13] >> 8 & 1, t.regs_expected[13] >> 9 & 1, t.regs_expected[13] >> 10 & 1, t.regs_expected[13] >> 11 & 1);
                printf("  Actual flags:   CF=%d PF=%d AF=%d ZF=%d SF=%d TF=%d IF=%d, DF=%d OF=%d\n", 
                    t.regs[i] & 1, t.regs[i] >> 2 & 1, t.regs[i] >> 4 & 1, t.regs[i] >> 6 & 1, 
                    t.regs[i] >> 7 & 1, t.regs[i] >> 8 & 1, t.regs[i] >> 9 & 1, t.regs[i] >> 10 & 1, t.regs[i] >> 11 & 1);
            }
            success = false;
            // if (failure == -1) failure = t.idx;
            break;
        }
    }
    tb.dbg_reg_rd = 0;

    // check ram values
    for (int i = ignore_memory; i < t.final.ram.size(); i++) {
        uint8_t dout;
        uint32_t addr = t.final.ram[i].first;
#if 1
        uint32_t dword = tb.z86_test->ram->mem[addr / 4];
        int off = addr % 4;
        dout = (dword >> (off * 8)) & 0xff;
#else
        tb.dbg_mem_rd = 1;
        tb.dbg_mem_addr = addr;
        step(); step();
        dout = tb.dbg_mem_dout;
#endif
        if (dout != t.final.ram[i].second) {
            printf("Ram %d MISMATCH: %02x != %02x\n", i, dout, t.final.ram[i].second);
            success = false;
            if (failure == -1) failure = t.idx;
            break;
        }
    }
    tb.dbg_mem_rd = 0;

    // check regs
    if (!success) {
        printf("Registers: AX=%04x BX=%04x CX=%04x DX=%04x SP=%04x BP=%04x SI=%04x DI=%04x \n", t.regs[0], t.regs[3], t.regs[1], t.regs[2], t.regs[4], t.regs[5], t.regs[6], t.regs[7]);
        printf("           CS=%04x SS=%04x DS=%04x ES=%04x IP=%04x \n", t.regs[8], t.regs[9], t.regs[10], t.regs[11], t.regs[12]);
        printf("Flags:     CF=%d PF=%d AF=%d ZF=%d SF=%d TF=%d IF=%d, DF=%d OF=%d\n", t.regs[13] & 1, t.regs[13] >> 2 & 1, t.regs[13] >> 4 & 1, t.regs[13] >> 6 & 1, 
                        t.regs[13] >> 7 & 1, t.regs[13] >> 8 & 1, t.regs[13] >> 9 & 1, t.regs[13] >> 10 & 1, t.regs[13] >> 11 & 1);

        std::cout << "=== Test case ===" << std::endl;
        std::cout << "  Bytes:        " << t.bytes << std::endl;
        std::cout << "  Initial regs: " << t.initial.regs << std::endl;
        std::cout << "  Initial ram:  " << t.initial.ram << std::endl;
        std::cout << "  Final regs:   " << t.final.regs << std::endl;
        std::cout << "  Final ram:    " << t.final.ram << std::endl;

        printf("\n");
        return false;
    } 
    printf("Test %s PASSED\n\n", t.name.c_str());
    return true;
}

#define OP(x) (test_file.find(x) != std::string::npos)

void run_json_test(std::string &test_file, char **argv, int argc, int off) {
    int start_test_idx = atoi(argv[off+1]);
    int end_test_idx = argc > off+2 ? atoi(argv[off+2]) : start_test_idx;

    // Shifts have undefined AF
    if (OP("D0") || OP("D1") || OP("D2") || OP("D3")) {
        ignore_mask |= 1 << 4;
        printf("Ignoring AF for D0/D1/D2/D3 tests\n");
    }
    if (test_file.find("D2") != std::string::npos || test_file.find("D3") != std::string::npos ||
        test_file.find("C0") != std::string::npos || test_file.find("C1") != std::string::npos) {
        ignore_mask |= 1 << 11;
        printf("Ignoring OF for D2/D3/C0/C1 tests\n");
    }

    if (test_file.find("F6.6") != std::string::npos || test_file.find("F7.6") != std::string::npos ||
        test_file.find("F6.7") != std::string::npos || test_file.find("F7.7") != std::string::npos) {
        ignore_mask = ~0;      // ignore all flags for DIV/IDIV
        ignore_memory = 2;     // ignore two bytes of memory - flags on stack
        printf("Ignoring all flags for F6.6/F7.6/F6.7/F7.7 tests and two bytes of memory (flags on stack)\n");
    } else if (test_file.find("F6") != std::string::npos || test_file.find("F7") != std::string::npos ||
               test_file.find("69") != std::string::npos || test_file.find("6B") != std::string::npos) {
        ignore_mask = ~(1 << 11) & ~(1 << 0);      // only test CF/OF
        printf("Ignoring all flags except CF/OF for MUL/IMUL tests\n");
    }
    if (test_file.find("27") != std::string::npos || test_file.find("2F") != std::string::npos) {
        ignore_mask |= (1 << 11);      // ignore OF for DAA/DAS
        printf("Ignoring OF for DAA/DAS tests\n");
    }
    if (test_file.find("37") != std::string::npos || test_file.find("3F") != std::string::npos) {
        ignore_mask = ~(1) & ~(1 << 4);      // tests only CF/AF for AAA/AAS
        printf("Testing only CF/AF for AAA/AAS tests\n");
    }
    if (test_file.find("D4") != std::string::npos || test_file.find("D5") != std::string::npos) {
        ignore_mask |= (1 << 11) | (1 << 4) | (1 << 0);      // ignore OF/AF/CF for AAM/AAD
        printf("Ignoring OF/AF/CF for AAM/AAD tests\n");
    }

    std::ifstream input_file(test_file);
    printf("Decoding JSON\n");
    std::vector<TestCase> tests = jsoncons::decode_json<std::vector<TestCase>>(input_file);
    
    printf("Simulation started\n");

    int total = 0, passed = 0;
    for (int i = start_test_idx; i <= end_test_idx; i++) {
        TestCase &t = tests[i];
        if (t.bytes.empty()) continue; 
        total++;
        bool success = run_test(t);
        if (success) passed++;
        else if (failure == -1) failure = i;
    }
    printf("Total tests: %d, passed: %d, failed: %d\n", total, passed, total - passed);
    if (failure != -1) {
        printf("Test %d failed\n", failure);
    }

    tb.cpu_reset = 1;
    step(); step();
}

// The .bin file contains the test program (64KB). We'll place it at F0000h - FFFFFh
// It contains a boot vector at FFFFEh. The execution starts at FFFFEh. It thens jumps
// to the main test program. We set DS and SS both to 1000h. So data generated by the 
// test program will be at 10000h.
void run_bin_test(std::string &test_file, std::string &result_file, int result_size) {
    std::ifstream input_file(test_file);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());

    trace_on = false;

    tb.reset = 1;      // reset whole system
    tb.cpu_reset = 1;  // reset CPU
    step(); step();
    tb.reset = 0;      // release system reset
    step(); step();

    // Load the program at F0000h - FFFFFh
    if (tb.clk_sys) step();    // make sure clk=0
    for (int i = 0; i < bytes.size(); i++) {
        tb.dbg_mem_wr = 1;
        tb.dbg_mem_addr = 0xF0000 + i;
        tb.dbg_mem_din = bytes[i];
        step(); step();
    }
    tb.dbg_mem_wr = 0;
    step(); step();

    // Set DS and SS to 0000h
    tb.dbg_reg_wr = 1;
    tb.dbg_reg_addr = 9;
    tb.dbg_reg_din = 0x0000;
    step(); step();
    tb.dbg_reg_addr = 10;
    tb.dbg_reg_din = 0x0000;
    step(); step();

    // Set CS:IP to F0000:FFF0h
    tb.dbg_reg_wr = 1;
    tb.dbg_reg_addr = 8;
    tb.dbg_reg_din = 0xF000;
    step(); step();
    tb.dbg_reg_addr = 12;
    tb.dbg_reg_din = 0xFFF0;
    step(); step();

    tb.dbg_reg_wr = 0;
    step(); step();

    trace_on = true;

    tb.cpu_reset = 0;  // start CPU 
    step(); step();

    // Run the program
    int cycles = 10000;
    run_program(cycles, true);

    if (tb.clk_sys) step();    // make sure clk=0
    tb.cpu_reset = 1;          // stop CPU
    step(); step();
    step(); step();

    // print the first 160 bytes of 0000:0000h
    std::ofstream ostream;

    if (result_file != "") {
        ostream.open(result_file, std::ios::binary | std::ios::out);
    }

    for (int i = 0; i < result_size; i++) {
        uint8_t dout;
        uint32_t addr = i;
#if 1
        uint32_t dword = tb.z86_test->ram->mem[addr / 4];
        int off = addr % 4;
        dout = (dword >> (off * 8)) & 0xff;
#else
        tb.dbg_mem_rd = 1;
        tb.dbg_mem_addr = addr;
        step(); step();
        dout = tb.dbg_mem_dout;
#endif

        if (i % 16 == 0) printf("%04x: ", i);
        printf("%02x ", dout);
        if (i % 16 == 15) printf("\n");
        if (result_file != "") {
            ostream.write((char*)&dout, 1);
        }
    }

    if (result_file != "") {
        ostream.close();
    }
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);

    if (argc < 2) {
        printf("Usage: %s [--no-trace] [--long] <test_file> <start_test_idx> [<end_test_idx>]\n", argv[0]);
        printf("               run 8088 single-instructions tests\n");
        printf("       %s <bin_test_file>\n", argv[0]);
        printf("               run 80186 short-program tests\n");
        return 1;
    }

    int off = 1;
    if (off < argc && std::string(argv[off]) == "--no-trace") {
        trace_on = false;
        off++;
    }

    if (off < argc && std::string(argv[off]) == "--long") {
        // long_test = true;
        test_length = 2000;
        off++;
    }

    printf("Starting simulation\n");
    if (trace_on) {
        trace = new VerilatedFstC;
        tb.trace(trace, 5);
        Verilated::traceEverOn(true);
        printf("Tracing to waveform.fst\n");
        trace->open("waveform.fst");
    }    

    std::string test_file = argv[off];
    std::string result_file = "";
    int result_size = 192;   // how many bytes to dump to result file
    if (argc > off+1) {
        result_file = argv[off+1];
    }
    if (argc > off+2) {
        result_size = atoi(argv[off+2]);
    }

    if (test_file.find(".json") != std::string::npos) {
        run_json_test(test_file, argv, argc, off);
    } else {
        run_bin_test(test_file, result_file, result_size);
    }

    // Cleanup
    if (trace_on) {
        trace->close();
        delete trace;
    }
    return 0;
} 