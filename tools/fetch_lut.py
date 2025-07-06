#!/usr/bin/env python3

# Generate fetch LUT: fetch_lut.svh

# instruction layout flags for each opcode
# {is_modrm, is_imm8, is_imm16, is_imm32}
# Source: https://wanker742126.neocities.org/new/dos_asm/ap07 (8086) and https://www.renesas.cn/zh/document/dst/80c286-datasheet (page 64-65)
def build_layout_flags():
    flags = [
        # 0       1       2       3       4       5       6       7       8       9       a       b       c       d       e       f       
        0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000, 0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000,   # 0
        0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000, 0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000,   # 1
        0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000, 0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000,   # 2
        0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000, 0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0010, 0b0000, 0b0000,   # 3
        0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000,   # 4
        0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000,   # 5
        0b0000, 0b0000, 0b1000, 0b1000, 0b0000, 0b0000, 0b0000, 0b1000, 0b0010, 0b1010, 0b0100, 0b1100, 0b0000, 0b0000, 0b0100, 0b0100,   # 6
        0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100,   # 7
        0b1100, 0b1010, 0b1100, 0b1100, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000,   # 8
        0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0001, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000,   # 9
        0b0010, 0b0010, 0b0010, 0b0010, 0b0000, 0b0000, 0b0000, 0b0000, 0b0100, 0b0010, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000,   # a
        0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0010, 0b0010, 0b0010, 0b0010, 0b0010, 0b0010, 0b0010, 0b0010,   # b
        0b1100, 0b1100, 0b0010, 0b0000, 0b1000, 0b1000, 0b1100, 0b1010, 0b0010, 0b0000, 0b0010, 0b0000, 0b0000, 0b0100, 0b0000, 0b0000,   # c
        0b1000, 0b1000, 0b1000, 0b1000, 0b0100, 0b0100, 0b0000, 0b0000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000, 0b1000,   # d
        0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0100, 0b0010, 0b0010, 0b0001, 0b0100, 0b0000, 0b0000, 0b0000, 0b0000,   # e
        0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b1000, 0b1000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b1000, 0b1000,   # f
        # 0f prefixed instructions
        0b1000, 0b1000, 0b1000, 0b1000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000, 0b0000]   # 10
    flags.extend([0]*16*15)
    return flags

# Register related control signals for DECODE
# Groups into E-related and G-related.
regctrl_WIDTH = 18
E_WIDTH = 7
def build_regctrl_lut():
    # Use a list comprehension to avoid aliasing between rows
    regctrl = [[0]*16 for _ in range(32)]
    
    Er    =1<<0     # E is specified by modrm[7:6] and modrm[2:0] (register or memory)
    Ea    =1<<1     # E is AX
    Esg   =1<<2     # E is segment register specified by opcode[5:4]
    Eop   =1<<3     # E is register specified by opcode[2:0]
    Eo    =1<<4     # E is specified by address in imm (A0-A3, MOV A,O)
    Ew    =1<<5     # E is write-only, no read
    Ez    =1<<6     # E is read-only, no write

    Gr    =1<<7     # G is specified by modrm[5:3]
    Ga    =1<<8     # G is AX (always 16-bit)
    Gc    =1<<9     # G is CX
    Gd    =1<<10    # G is DX
    Gsp   =1<<11    # G is SP
    Gsg   =1<<12    # G is segment register specified by modrm[5:3]
    Gi    =1<<13    # G is IMM, width is specified W
    Gw    =1<<14    # G is write-only, no read
    Gz    =1<<15    # G is read-only, no write

    W     =1<<16    # Word instruction, otherwise byte
    STACK_OP     =1<<17    # Using and writing SP
    
    #            0/8            1/9            2/a            3/b            4/c            5/d            6/e            7/f
    regctrl[0] =[Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       Esg|Gsp|Ez|W,  Esg|Gsp|Ew|W,  # 0?: ADD PUSH/POP ES
                 Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       Esg|Gsp|Ez|W,  0            ] #     OR  PUSH CS
    regctrl[1] =[Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       Esg|Gsp|Ez|W,  Esg|Gsp|Ew|W,  # 1?: ADC PUSH/POP SS
                 Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       Esg|Gsp|Ez|W,  Esg|Gsp|Ew|W ] #     SBB PUSH/POP DS
    regctrl[2] =[Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       0,             Ea|W,          # 2?: AND ES DAA
                 Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       0,             Ea|W         ] #     SUB CS DAS
    regctrl[3] =[Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       0,             Ea|W,          # 3?: XOR SS AAA
                 Er|Gr|Gz,      Er|Gr|Gz|W,    Er|Gr|Ez,      Er|Gr|Ez|W,    Ea|Gi,         Ea|Gi|W,       0,             Ea|W         ] #     CMP DS AAS
    regctrl[4] =[Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         # 4?: INC
                 Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W,         Eop|W        ] #     DEC
    regctrl[5] =[Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  Eop|Gsp|Ez|W,  # 5?: PUSH
                 Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W,  Eop|Gsp|Ew|W ] #     POP
    regctrl[6] =[0,             0,             Er|Ez|W,       0,             0,             0,             0,             0,             # 6?: PUSHA, POPA, BOUND
                 Gsp,           Er|Gr|Ez|Gw|W, Gsp,           Er|Gr|Ez|Gw|W, 0,             0,             0,             0            ]      
                                                                                                                                         # 7?: Jcc: all 0
    regctrl[8] =[Er|Gi,         Er|Gi|W,       Er|Gi,         Er|Gi|W,       Er|Gr|Ez|Gz,   Er|Gr|Ez|Gz|W, Er|Gr,         Er|Gr|W,       # 8?: GRP1 TEST XCHG
                 Er|Gr|Ew|Gz,   Er|Gr|Ew|Gz|W, Er|Gr|Ez|Gw,   Er|Gr|Ez|Gw|W, Er|Gsg|Ew|Gz|W,Er|Gr|Ew|Gw|W, Er|Gsg|Ez|Gw|W,Er|Ew|Gsp|W  ] #     MOV LEA POP
    regctrl[9] =[0,             Eop|Ga|W,      Eop|Ga|W,      Eop|Ga|W,      Eop|Ga|W,      Eop|Ga|W,      Eop|Ga|W,      Eop|Ga|W,      # 9?: NOP XCHG
                 Ea|W,          Ea|W,          0,             0,             0,             0,             Ea|Ez|W,       Ea|W         ] #     CBD CWD SAHF LAHF
    regctrl[10]=[Eo|Ga|Gw,      Eo|Ga|Gw|W,    Eo|Ga|Gz,      Eo|Ga|Gz|W,    0,             0,             0,             0,             # A?: MOV, MOVS, CMPS
                 Ea|Gi|Ez,      Ea|Gi|Ez|W,    Ea|Ez,         Ea|Ez|W,       Ea|Ew,         Ea|Ew,         Ea|Ez,         Ea|Ez|W      ] #     TEST, STOS, LODS, SCAS
    regctrl[11]=[Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     Eop|Gi|Ew,     # B?: MOV reg,Ib
                 Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W,   Eop|Gi|Ew|W  ] #     MOV reg,Iv
    regctrl[12]=[Er,            Er|W,          Gi|W,          0,             Er|Gr|Ez|Gw|W, Er|Gr|Ez|Gw|W, Er|Gi|Ew,      Er|Gi|Ew|W,    # C?: RET, LES, LDS, MOV E,I, 
                 0,             0,             0,             0,             0,             0,             0,             0            ] #     RETF, INT, all microcoded
    regctrl[13]=[Er,            Er|W,          Er|Gc|Gz,      Er|Gc|Gz|W,    Ea|W,          Ea|W,          0,             Ga|W,          # D?: GRP2, AAM, AAD, XLAT
                                         #                                                                                AL=[BX+AL]        <-- special cases
                 0,             0,             0,             0,             0,             0,             0,             0            ] #     ESC
    regctrl[14]=[Gc|W,          Gc|W,          Gc|W,          Gc|W,          Ea|Ew,         Ea|Ew|W,       Ea|Ez,         Ea|Ez|W,       # E?: LOOP, JCXZ, IN, OUT
                 Gsp,           0,             0,             0,             Ea|Gd|Ew|Gz,   Ea|Gd|Ew|Gz|W, Ea|Gd|Ez,      Ea|Gd|Ez|W   ] #     CALL, JMP, IN, OUT
    regctrl[15]=[0,             0,             0,             0,             0,             0,             Er|Ga,         Er|Ga|W,       # F?: LOCK REP HLT CMC GRP3
                 0,             0,             0,             0,             0,             0,             Er,            Er|Gsp|W     ] #     STI CLI

    # 0f prefixed instructions: no regctrl for now

    STACK_OPS = [0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
                 0x06, 0x07, 0x0E, 0x16, 0x17, 0x1E, 0x1F, 0x9A, 0x9B, 0x9C, 0x9D, 
                 0xC0, 0xC1, 0xC2, 0xC3, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
                 0xE8, 0xFF, 0x60, 0x61]        # 0xFF is not all stack ops
    for i in STACK_OPS:
        regctrl[i//16][i%16] |= STACK_OP
    
    return regctrl

layout = build_layout_flags()   # 4 bits
regctrl = build_regctrl_lut()   # 8 bits (4 for E, 4 for G)

with open("fetch_lut.svh", "w") as f:
    f.write("// Automatically generated. Do not edit.\n")
    f.write("// Fetch LUT. {regctrl_g, regctrl_e, layout[3:0]}. See tools/fetch_lut.py\n")
    f.write(f"localparam [{4+regctrl_WIDTH-1}:0] FETCH_LUT [0:511] = '{{\n")
    f.write("//")
    w = regctrl_WIDTH + 4 + 9
    for i in range(32):
        f.write(f"{i:{w-8}x}")
        f.write("       ")
    f.write("\n")
    for i in range(512):
        comma = "," if i < 511 else ""
        layout_bits = layout[i]
        regctrl_bits = regctrl[i//16][i%16]
        G_WIDTH = regctrl_WIDTH - E_WIDTH
        regctrl_e_bits = regctrl_bits & ((1 << E_WIDTH) - 1)
        regctrl_g_bits = (regctrl_bits >> E_WIDTH)
        f.write(f"{regctrl_WIDTH+4}'b{regctrl_g_bits:0{G_WIDTH}b}_{regctrl_e_bits:0{E_WIDTH}b}_{layout_bits:04b}{comma} ")
        if (i + 1) % 16 == 0:
            f.write(f"// {i//16:x}\n")
    f.write("};\n")

