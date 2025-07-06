;;; z86 microcode
;;;    tools/ucode.py ucode.asm
;;; generates ucode_rom.svh and ucode_entry.svh

;;;  ---------- JMP/CALL/RET ---------------------------------

@NOP
    .end

@CALL_NEAR                 ; E8: CALL Jv
    PUSH  IP_AFTER         ; src selector defined in table
    BR_REL16               ; ip ← ip_after + disp16
    .end                   ; auto-sets stop=1

@CALL_RM                   ; FF.2: CALL Ev
    PUSH  IP_AFTER
    LOAD      0            ; read target word
    BR_ABS16               ; ip ← tmp_lo
    .end

@CALL_FAR                  ; 9A: CALLF
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     1           ; arg=1: cs:ip ← e2_disp
    .end

@CALL_EP                   ; FF.3: CALL Ep
    PUSH  CS               ; save current CS
    PUSH  IP_AFTER         ; save return address
    LOAD       0           ; read target offset
    LOAD       1           ; read target segment
    BR_FAR     0           ; arg=0: cs:ip ← tmp_hi:tmp_lo
    .end

@JMP_RM                    ; FF.4: JMP Ev
    LOAD       0           ; read target offset word → tmp_lo
    BR_ABS16               ; IP ← tmp_lo   (segment unchanged)
    .end

@JMP_EP                    ; FF.5: JMP Ep
    LOAD       0           ; read target offset
    LOAD       1           ; read target segment
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@RET_NEAR                  ; C3/C1: RETN
    POP   0                ; pop return address into tmp_lo
    BR_ABS16               ; branch to return address
    .end

@RET_NEAR_IMM              ; C2/C0: RETN Iw
    POP   0                ; pop return address
    ADJSP 0                ; add imm16 to SP
    BR_ABS16               ; branch to return address
    .end

@RET_FAR                   ; CB/C9: RETF
    POP   0                ; pop return offset into tmp_lo
    POP   1                ; pop return segment into tmp_hi
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@RET_FAR_IMM               ; CA/C8: RETF Iw
    POP   0                ; pop return offset into tmp_lo
    POP   1                ; pop return segment into tmp_hi
    ADJSP 0                ; add imm16 to SP
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@IRET                       ; CF
    POP     0               ; tmp_lo  
    POP     1               ; tmp_hi 
    POP     2               ; tmp_flags
    WR_FLAGS                ; FLAGS ← tmp_flags
    BR_FAR  0               ; cs:ip ← tmp_hi:tmp_lo
    .end


;;;  ---------- INT instructions --------------------------------------

@INT3                      ; CC: INT3
    PUSH  FLAGS
    GETVEC_OFF      3      ; vector 3, first get offset, also clears IF and TF
    GETVEC_SEG      3      ; then get segment
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@INT_IMM                   ; CD: INT Ib
    PUSH  FLAGS
    GETVEC_OFF      15     ; vector from immediate
    GETVEC_SEG      15
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@INTO                      ; CE: INTO
    TESTF   OF             ; mc_cond = OF
    JF     6              ; IF OF==0  →  skip next 6 µ-ops
    PUSH  FLAGS
    GETVEC_OFF      4      ; vector 4 (overflow)
    GETVEC_SEG      4          
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

@INT1                      ; F1: INT1
    PUSH  FLAGS
    GETVEC_OFF      1      ; vector 1
    GETVEC_SEG      1          
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     0           ; cs:ip ← tmp_hi:tmp_lo
    .end

;;; ----------------  String instructions  -----------------------------

;----------------  MOVSB / REP_MOVSB -----------------------------

@MOVSB                     ; A4
    LOAD      0b0110       ; byte, SI++, DS:SI  → tmp_lo
    STORE     0b0110       ; byte, DI++, tmp_lo → ES:DI
    .end 
 
@REP_MOVSB
    TESTZX
    JT        4
    LOAD      0b0110       ; byte, SI++, DS:SI  → tmp_lo
    STORE     0b0110       ; byte, DI++, tmp_lo → ES:DI
    DEC       CX
    J         -6           ; CX != 0, back to LOAD
    .end 

;----------------  MOVSW / REP_MOVSW -----------------------------

@MOVSW                     ; A5
    LOAD      0b1110       ; word, SI++, DS:SI  → tmp_lo
    STORE     0b1110       ; word, DI++, tmp_lo → ES:DI
    .end 

@REP_MOVSW
    TESTZX
    JT        4
    LOAD      0b1110       ; word, SI++, DS:SI  → tmp_lo
    STORE     0b1110       ; word, DI++, tmp_lo → ES:DI
    DEC       CX
    J         -6           ; CX != 0, back to LOAD
    .end 
 
;----------------  CMPSB / REPNE_CMPSB / REPE_CMPSB -----------------------------

@CMPSB                     ; A6
    LOAD      0b0110       ; byte, SI++, DS:SI → tmp_lo
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    STR_CMP   0            ; 0 = byte
    .end 

@REPNE_CMPSB
    TESTZX
    JT        6            ; CX == 0, go to .end
    LOAD      0b0110       ; byte, SI++, DS:SI → tmp_lo
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    DEC       CX
    STR_CMP   0            ; 0 = byte
    TESTF     ZF           ; 
    JF        -8           ; ZF == 0, back to beginning
    .end 
 
@REPE_CMPSB
    TESTZX
    JT        6            ; CX == 0, go to .end
    LOAD      0b0110       ; byte, SI++, DS:SI → tmp_lo
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    DEC       CX
    STR_CMP   0            ; 0 = byte
    TESTF     ZF
    JT        -8           ; ZF == 1, back to beginning
    .end 

;----------------  CMPSW / REPNE_CMPSW / REPE_CMPSW -----------------------------

@CMPSW                     ; A7
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    STR_CMP   1            ; word
    .end 
 
@REPNE_CMPSW                     ; A7
    TESTZX
    JT        6            ; CX == 0, go to .end
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    STR_CMP   1            ; word
    TESTF     ZF
    JF        -8           ; ZF == 0, back to beginning
    .end 

@REPE_CMPSW
    TESTZX
    JT        6            ; CX == 0, go to .end
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    STR_CMP   1            ; word
    TESTF     ZF
    JT        -8           ; ZF == 1, back to beginning
    .end 

@REP_CMPSW
    TESTZX
    JT        5            ; CX == 0, goto .end
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    STR_CMP   1            ; word
    J         -7
    .end 

;----------------  STOSB / REP_STOSB -----------------------------

@STOSB                     ; AA
    STORE     0b0111       ; byte, DI++, AL → [ES:DI]
    .end 
 
@REP_STOSB
    TESTZX
    JT        3            ; CX == 0, goto .end
    DEC       CX
    STORE     0b0111       ; byte, DI++, AL → [ES:DI]
    J         -5
    .end 

;----------------  STOSW / REP_STOSW -----------------------------

@STOSW                     ; AB
    STORE     0b1111       ; word, DI++, AX → [ES:DI]
    .end 
 
@REP_STOSW
    TESTZX
    JT        3            ; CX == 0, goto .end
    DEC       CX
    STORE     0b1111       ; word, DI++, AX → [ES:DI]
    J         -5
    .end 

;----------------  LODSB / REP_LODSB -----------------------------

@LODSB                     ; AC
    LOAD      0b0110       ; byte, SI++, DS:SI → tmp_lo
    WR_REG    AL           ; AL ← tmp_lo
    .end 

@REP_LODSB
    TESTZX
    JT        4            ; CX == 0, goto .end
    LOAD      0b0110       ; byte, SI++, DS:SI → tmp_lo
    DEC       CX
    WR_REG    AL           ; AL ← tmp_lo
    J         -6
    .end 

;----------------  LODSW / REP_LODSW -----------------------------

@LODSW                     ; AD
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    WR_REG    AX           ; AX ← tmp_lo
    .end 
 
@REP_LODSW
    TESTZX
    JT        4            ; CX == 0, goto .end
    LOAD      0b1110       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    WR_REG    AX           ; AX ← tmp_lo
    J         -6
    .end 

;----------------  SCASB / REPNE_SCASB / REPE_SCASB -----------------------------

@SCASB                     ; AE
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    STR_CMP   0 
    .end 

@REPNE_SCASB
    TESTZX
    JT        5            ; CX == 0, go to .end
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    DEC       CX
    STR_CMP   0 
    TESTF     ZF
    JF        -7           ; ZF == 0, back to beginning
    .end 

@REPE_SCASB
    TESTZX
    JT        5            ; CX == 0, go to .end
    LOAD      0b0111       ; byte, DI++, ES:DI → tmp_hi
    DEC       CX
    STR_CMP   0 
    TESTF     ZF
    JT        -7           ; ZF == 1, back to beginning
    .end

;----------------  SCASW / REPNE_SCASW / REPE_SCASW -----------------------------

@SCASW                     ; AF
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    STR_CMP   1
    .end

@REPNE_SCASW
    TESTZX
    JT        5            ; CX == 0, go to .end
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    STR_CMP   1
    TESTF     ZF
    JF        -7           ; ZF == 0, back to beginning
    .end 

@REPE_SCASW
    TESTZX
    JT        5            ; CX == 0, go to .end
    LOAD      0b1111       ; word, SI++, DS:SI → tmp_lo
    DEC       CX
    STR_CMP   1
    TESTF     ZF
    JT        -7           ; ZF == 1, back to beginning
    .end 

;----------------  LES/LDS -----------------------------

@LES
    LOAD      0            ; @EA → tmp_lo
    LOAD      1            ; @EA+2 → tmp_hi
    WR_REG    REG          ; regs[modrm[5:3]] ← tmp_lo
    WR_SEG    0            ; ES ← tmp_hi
    .end 

@LDS
    LOAD      0            ; @EA → tmp_lo
    LOAD      1            ; @EA+2 → tmp_hi
    WR_REG    REG          ; regs[modrm[5:3]] ← tmp_lo
    WR_SEG    3            ; DS ← tmp_hi
    .end

;;; ----------------  Stack instructions  -----------------------------

@PUSHF                     ; PUSHF 9C
    PUSH      FLAGS
    .end

@POPF                      ; POPF 9D
    POP       TMP_FLAGS    ; pop to tmp_flags
    WR_FLAGS               ; FLAGS ← tmp_flags
    .end

@PUSH_R16                  ; PUSH Ev (FF.6/7 register form)
    PUSH      RM           ; push regs[modrm[2:0]]
    .end

@PUSH_M16                  ; PUSH Ev (FF.6/7 memory form)
    LOAD      0            ; @EA → tmp_lo
    PUSH      TMP_LO       ; push tmp_lo
    .end

;;; ----------------  Multiplication / Division  -----------------------------

@MUL8                    ; MUL r/m8  (F6 /4), unsigned multiplication
    LOAD      0          ; r/m → tmp_lo (byte)
    MULU      0          ; arg[0]=0  (byte)  
    .end

@MUL16                   ; MUL r/m16 (F7 /4), unsigned multiplication
    LOAD      8          ; r/m → tmp_lo (word)
    MULU      1          ; arg[0]=1  (word)  
    .end

@IMUL8                   ; IMUL r/m8 (F6 /5), signed multiplication
    LOAD      0
    MULS      0          ; signed, byte  
    .end

@IMUL16                  ; IMUL r/m16 (F7 /5), signed multiplication
    LOAD      0
    MULS      1          ; signed, word   
    .end

;;; -----------------------------------------------------------
;;; DIV / IDIV  (F6 /6-7  F7 /6-7)
;;; -----------------------------------------------------------

@DIV8        ; F6 /6
    LOAD      0              ; divisor  -> tmp_lo
    DIV       0b00           ; width=0, signed=0
    JT       1              ;
    .end
; trigger INT0 on overflow
    GETVEC_OFF      0        ; vector 0, division trap
    GETVEC_SEG      0        ; then get segment
    PUSH  FLAGS
    PUSH  CS
    PUSH  IP_AFTER           ; for both 8086/80186, IP_AFTER is pushed. 80286 pushes IP_THIS
    BR_FAR     0             ; cs:ip ← tmp_hi:tmp_lo
    .end

@DIV16       ; F7 /6
    LOAD      0
    DIV       0b01           ; width=1, signed=0
    JT       1              ;
    .end
; trigger INT0 on overflow
    GETVEC_OFF      0        ; vector 0, division trap
    GETVEC_SEG      0        ; then get segment
    PUSH  FLAGS
    PUSH  CS
    PUSH  IP_AFTER           
    BR_FAR     0             ; cs:ip ← tmp_hi:tmp_lo
    .end

@IDIV8       ; F6 /7
    LOAD      0
    DIV       0b10           ; width=0, signed=1
    JT       1              ;
    .end
; trigger INT0 on overflow
    GETVEC_OFF      0        ; vector 0, division trap
    GETVEC_SEG      0        ; then get segment
    PUSH  FLAGS
    PUSH  CS
    PUSH  IP_AFTER           
    BR_FAR     0             ; cs:ip ← tmp_hi:tmp_lo
    .end

@IDIV16      ; F7 /7
    LOAD      0
    DIV       0b11           ; width=1, signed=1
    JT       1              ;
    .end
; trigger INT0 on overflow
    GETVEC_OFF      0        ; vector 0, division trap
    GETVEC_SEG      0        ; then get segment
    PUSH  FLAGS
    PUSH  CS
    PUSH  IP_AFTER
    BR_FAR     0             ; cs:ip ← tmp_hi:tmp_lo
    .end


;;; ----------------  BCD/ASCII adjust instructions  -----------------------------

; ─────────────────────────  Adjust family  ─────────────────────────

@DAA          ; 27
    ADJ   0  
    .end

@DAS          ; 2F
    ADJ   1 
    .end

@AAA          ; 37
    ADJ   2 
    .end

@AAS          ; 3F
    ADJ   3 
    .end

@AAM          ; D4  imm8
    ADJ       4 
    JT       1              ; ADJ sets cond if divide-by-0
    .end
    GETVEC_OFF      0        ; vector 0, division trap
    GETVEC_SEG      0        ; then get segment
    PUSH  FLAGS
    PUSH  CS
    PUSH  IP_AFTER           
    BR_FAR     0             ; cs:ip ← tmp_hi:tmp_lo
    .end


@AAD          ; D5  imm8
    AAD       ; uses separate µ-op for better performance as there's a 8x8 multiplication
    .end


@HLT
    HALT
    .end


;;; ─────────────────────── 80286 extensions ────────────────────────

; ─────────── 60 / 61  PUSHA / POPA ───────────────────────────────

@PUSHA                  ; 60
    READ     SP         ; tmp_lo ← current SP
    PUSH     AX
    PUSH     CX
    PUSH     DX
    PUSH     BX
    PUSH     TMP_LO      ; original SP
    PUSH     BP
    PUSH     SI
    PUSH     DI
    .end

@POPA                     ; 61 (reverse order, skip SP)
    POP      DI
    POP      SI
    POP      BP
    POP      TMP_LO           ; throw-away word = saved SP
    POP      BX
    POP      DX
    POP      CX
    POP      AX
    .end


; ─────────── 62  BOUND Ev,Gv  (interrupt 5 on error) ──────────────

@BOUND                  ; 62
    LOAD   0                 ; lower bound  → tmp_lo
    LOAD   1                 ; upper bound  → tmp_hi
    IN_RANGE                 ; sets cond = (tmp_lo <= Gv <= tmp_hi)
    JT     5                 ; in range → .end
; --- fault path ---------------------------------------------------
    GETVEC_OFF 5             ; vector 5  (bound-range exceeded)
    GETVEC_SEG 5
    PUSH     CS
    PUSH     IP_AFTER
    BR_FAR   0
    .end


; ─────────── 63  ARPL Wr,Ev  (dest is r/m16) ──────────────────────

; TODO: implement ARPL
@ARPL                   ; 63
    ; LOAD   0                ; r/m16 → tmp_lo         (mem or reg)
    ; ;RD_REG   G, TMP_HI     ; src register (Wr) → tmp_hi
    ; ARPL_OP                 ; **NEW** do RPL adjust, sets ZF
    ; STORE_RM TMP_LO         ; write back to r/m16
    .end


; ─────────── 69 / 6B  IMUL Gv,Ev,imm16/imm8 ─────────────────────────────
@IMUL_IW                ; 69
    LOAD     0                 ; tmp_lo ← Ev
    MULS     3                 ; G ← tmp_lo * imm16
    .end

@IMUL_IB                ; 6B
    LOAD     0
    MULS     2                 ; G ← tmp_lo * imm8
    .end

; ─────────── 6C / 6D / 6E / 6F  String I/O ‐ INxx / OUTxx ─────────

@INSB                   ; 6C
    IN      BYTE             ; in DX → tmp_lo
    STORE   0b0110           ; byte, DI++, tmp_lo → ES:DI
    .end

@INSW                   ; 6D
    IN      WORD
    STORE   0b1110           ; word, DI++, tmp_lo → ES:DI
    .end

@OUTSB                  ; 6E
    LOAD    0b0110           ; byte, SI++, DS:SI → tmp_lo
    OUT     BYTE             ; tmp_lo → DX
    .end

@OUTSW                  ; 6F
    LOAD    0b1110           ; word, SI++, DS:SI → tmp_lo
    OUT     WORD
    .end


; ─────────── C8 / C9  ENTER / LEAVE ───────────────────────────────
; Stack frame layout after ENTER Iw,2
;                                 caller's frame
;                                        |
;          +---------------------------+ |  
;          |     Caller's frame ptr    | |  ← pushed by ENTER
;          +---------------------------+ |
; BP →   +>|      Static link 1        |-+ 
;        | +---------------------------+
;        +-|      Static link 2        | 
;          +---------------------------+
;          |      local variable       |  
;          +---------------------------+
;          |      local variable       | 
;          +---------------------------+
;          |                           |    ← SP after ENTER
;          +---------------------------+  

@ENTER                       ; C8   ENTER Iw,Ib
    PUSH     BP
    READ     ENTER_IB        ; tmp_hi ← Ib
    READ     SP              ; tmp_lo ← SP
    WR_REG   BP              ; BP ← SP

; TODO: push actual static links. Now links to current frame is pushed.
;LOOP
    TESTZX   TMP_HI          
    JT       3
    PUSH     BP
    DEC      TMP_HI
    J        -5              ; LOOP

    ADJSP    SUB_IW          ; SP -= Iw
    .end

@LEAVE                       ; C9
    READ     BP              
    WR_REG   SP              ; SP ← BP
    POP      BP
    .end

