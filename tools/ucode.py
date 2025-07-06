#!/usr/bin/env python3

"""
Microcode assembler for z86. This generates both the microcode ROM and entry labels
- nand2mario, 6/2025
"""

import re, sys, json, textwrap
OPCODES = {           # 6-bit values must match execute.sv
    'NOP'       : 0,
    'PUSH'      : 1,
    'POP'       : 2,
    'ADJSP'     : 3,
    'GETVEC_OFF': 4,
    'GETVEC_SEG': 5,
    'LOAD'      : 6,    # load from EA, [DS:SI] or [ES:DI], optionally increment SI/DI
    'STORE'     : 7,    # store to EA or [ES:DI], optionally increment DI
    'BR_REL16'  : 8,
    'BR_ABS16'  : 9,
    'BR_FAR'    : 10,
    'TESTF'     : 11,
    'TESTZX'    : 12,
    'J'         : 13,   # microcode jump (relative -8 to 7 micro-ops, counting from next micro-op)
    'JT'        : 14,   # microcode jump if true 
    'JF'        : 15,   # microcode jump if false
    'READ'      : 16,   # read register or immediate to tmp_lo
    'WR_FLAGS'  : 17,
    'WR_REG'    : 18,
    'WR_SEG'    : 19,
    'STR_CMP'   : 20,   # ⇢ AL/AX vs tmp_lo   or   tmp_lo vs tmp_hi   → flags
    'DEC'       : 21,   # decrement a register, clears CF, other flags unchanged
    'MULU'      : 22,
    'MULS'      : 23,
    'DIV'       : 24,   # AL/AX / tmp_lo → AX, DX/AH ← remainder
    'ADJ'       : 25,   # adjust AL/AX by BCD/ASCII
    'AAD'       : 26,
    'IN_RANGE'  : 27,   # test if tmp_lo <= Gv <= tmp_hi
    'IN'        : 28,
    'OUT'       : 29,
    'HALT'      : 30,
}

UINST_FMT = "    `U({op:2d},4'h{arg:1x},{stop}){comma} // {cmt}"

ARGS = {
    'ADJSP':  {'ADD_IW':0, 'SUB_IW':1},
    'LOAD':   {'RM': 8,'RM_HI':9,'DS_SI':14,'ES_DI':15},
    'PUSH':   {'CS':0,'IP_THIS':1,'IP_AFTER':2,'RM':3,'FLAGS':4,'TMP_LO':5,
               'AX':8,'CX':9,'DX':10,'BX':11,'SP':12,'BP':13,'SI':14,'DI':15},    # for PUSHA
    'POP' :   {'TMP_LO':0,'TMP_HI':1,'TMP_FLAGS':2,
               'AX':8,'CX':9,'DX':10,'BX':11,'SP':12,'BP':13,'SI':14,'DI':15},    # for POPA
    'READ':   {'SP':4,'BP':5,'ENTER_IB':8},
    'WR_REG': {'AX':0,'AL':8,'REG':10,'SP':4,'BP':5},
    'TESTZX': {'CX':0,'TMP_HI':1},
    'TESTF':  {'CF':0,'PF':1,'AF':2,'ZF':3,'SF':4,'OF':5},
    'DEC':    {'CX':1,'TMP_HI':8},
    'IN':     {'BYTE':0,'WORD':1},
    'OUT':    {'BYTE':0,'WORD':1},
}

def parse(src_text):
    pc = 0
    rom = {}
    labels = {}
    first_after_label = False
    for ln,line in enumerate(src_text.splitlines(),1):
        line = re.sub(r';.*','',line).strip()
        if not line: continue
        if line.startswith('@'):
            lbl = line[1:]
            if lbl in labels: sys.exit(f"Duplicate label {lbl} L{ln}")
            labels[lbl] = pc
            first_after_label = True
            continue
        if line.startswith('.org'):
            pc = int(line.split()[1],0)
            continue
        if line.startswith('.end') or line.startswith('.stop'):
            rom[pc] = ('NOP',0,1,f'{ln}: {line.strip()}')
            pc += 1
            continue
        # normal instruction
        m = re.match(r'(\w+)(\s+([\w\d\+\-]+))?',line)
        op,arg = m.group(1), m.group(3)
        op = op.upper()
        arg = arg.upper() if arg else None
        if op in ARGS:
            arg_map = ARGS[op]
        else:
            arg_map = {}
        if arg in arg_map:
            argv = arg_map[arg]
        else:
            try:
                argv = int(arg,0) if arg else 0
            except ValueError:
                sys.exit(f"Invalid argument {arg} L{ln}")
        # label_str = f'({lbl})' if first_after_label else ''
        # first_after_label = False
        if argv < -8:
            sys.exit(f"Invalid argument {arg} L{ln}")
        if argv < 0:        # arg is 4-bit
            argv = 16+argv
        rom[pc] = (op,argv,0,f"{ln}: {line.strip()}")
        pc += 1
    return rom, labels

def emit_sv(rom, depth=512):
    lines = []
    pc_to_lbl = {v:k for k,v in lbl.items()}
    for pc in range(depth):
        if pc in pc_to_lbl:
            lines.append(f"    // {pc:02X} {pc_to_lbl[pc]}")
        comma = ',' if pc < depth-1 else ''
        if pc in rom:
            op,arg,stop,cmt = rom[pc]
            lines.append(UINST_FMT.format(op=OPCODES[op],arg=arg,stop=stop,cmt=cmt,comma=comma))
        else:
            lines.append(UINST_FMT.format(op=0, arg=0, stop=1, cmt='empty',comma=comma))
    return "const uinstr_t microcode_rom [512] = '{\n" + "\n".join(lines) + "\n};"

def emit_labels(labels):
    out  = "// Auto-generated – do not edit\n"
    for k,v in labels.items():
        out += f"localparam [8:0] UENTRY_{k} = 9'h{v:02X};\n"
    return out

if __name__ == "__main__":
    txt = open(sys.argv[1]).read()
    rom,lbl = parse(txt)
    with open("ucode_rom.svh","w") as f:
        f.write("// Auto-generated – do not edit\n")
        for op in OPCODES:
            f.write(f"localparam [5:0] MC_{op} = {OPCODES[op]};\n")
        f.write(emit_sv(rom))
    with open("ucode_entry.svh","w") as f:
        f.write(emit_labels(lbl))
