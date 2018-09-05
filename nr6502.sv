/*******************************************************************************
*                     _   _ ____   __  ____   ___ ____                         *
*                    | \ | |  _ \ / /_| ___| / _ \___ \                        *
*                    |  \| | |_) | '_ \___ \| | | |__) |                       *
*                    | |\  |  _ <| (_) |__) | |_| / __/                        *
*                    |_| \_|_| \_\\___/____/ \___/_____|  v. 1.0.1             *
*                                                                              *
*             A SystemVerilog implementation of Bender's brain.                *
*                                                                              *
* Tested successfully on Apple-1 and Commodore PET 2001, on an Artix-7 FPGA.   *
*                                                                              *
* It passes the Klaus Dormann's test suite (interrupt & functionnal test with  *
* decimal mode).                                                               *
* (https://github.com/Klaus2m5/6502_65C02_functional_tests)                    *
*                                                                              *
* Known limitations compared to a real 6502:                                   *
*  - Undocumented flags of decimal mode are not implemented                    *
*  - Undocumented instructions are not implemented (executed as NOP)           *
*  - Not cycle accurate                                                        *
*  - No multi-phase clock.                                                     *
*                                                                              *
* Paris, 08/2018 - Nicolas Robin                                               *
*                                                                              *
################################################################################
# Copyright 2018 Nicolas Robin                                                 #
#                                                                              #
# Redistribution and use in source and binary forms, with or without           #
# modification, are permitted provided that the following conditions are met:  #
#                                                                              #
# 1. Redistributions of source code must retain the above copyright notice,    #
# this list of conditions and the following disclaimer.                        #
#                                                                              #
# 2. Redistributions in binary form must reproduce the above copyright notice, #
# this list of conditions and the following disclaimer in the documentation    #
# and/or other materials provided with the distribution.                       #
#                                                                              #
# 3. Neither the name of the copyright holder nor the names of its             #
# contributors may be used to endorse or promote products derived from this    #
# software without specific prior written permission.                          #
#                                                                              #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   #
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE    #
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR          #
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         #
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS     #
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      #
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)      #
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   #
# POSSIBILITY OF SUCH DAMAGE.                                                  #
##############################################################################*/

module nr6502 (
    // All output signals are synchronous with clk.
    // All input signals are assumed to be synchronous with clk.
    input clk,                      // CPU Clock
    input reset,                    // Synchronous Reset (one cycle needed)
    input [7:0] data_in,            // 8-bits Data Bus (input)
    output logic [7:0] data_out,    // 8-bits Data Bus (output)
    output logic [15:0] addr,       // 16-bits Address bus
    output logic r_w,               // Read / #Write
    input so,                       // Set Overflow, positive edge-sensitive
    input irq,                      // Positive level-sensitive
    input nmi,                      // Positive edge-sensitive
    input rdy,                      // Halt the CPU if low
    output sync,                    // High on fetch opcode state

    output invalid_op               // High = undocumented instruction fetched
);

//******************************************************************************
// Registers, wires and types
//******************************************************************************
logic [7:0] a;                              // Accumulator
logic [7:0] x, y;                           // Index X/Y
logic [15:0] pc;                            // PC
logic [7:0] s;                              // Stack pointer
logic n, v, d, i, z, c;                     // Processor Status bits
logic [7:0] iw;                             // Internal Working register
logic ifl;                                  // Internal Flag
logic [15:0] ip;                            // Internal Pointer register

// Interrupt
logic nmi_pending;                          // NMI edge detector / pending
wire int_pending;                           // NMI or IRQ on next T0

// Set Overflow edge detector / pending
logic so_pending;

// ALU
logic [7:0] alu_i1, alu_i2;         // ALU 8 bits inputs
logic alu_i3;                       // ALU 1 bit input
logic [7:0] alu_q;                  // ALU 8 bits ouput
logic alu_n, alu_c, alu_z, alu_v;   // ALU Flags output
enum logic [3:0] {
    OP_ADD, OP_ADD_BCD,
    OP_SUB, OP_SUB_BCD,
    OP_ROR, OP_ROL,
    OP_AND, OP_OR, OP_XOR,
    OP_IDT,
    OP_DONTCARE = 4'bX
} alu_op;

// Instruction decoder
typedef enum bit[5:0] {
    INS_UNDEFINED,
    ADC, AND, ASL, BCC, BCS, BEQ, BIT, BMI, BNE, BPL, BRK, BVC, BVS, CLC, CLD,
    CLI, CLV, CMP, CPX, CPY, DEC, DEX, DEY, EOR, INC, INX, INY, JMP, JSR, LDA,
    LDX, LDY, LSR, NOP, ORA, PHA, PHP, PLA, PLP, ROL, ROR, RTI, RTS, SBC, SEC,
    SED, SEI, STA, STX, STY, TAX, TAY, TSX, TXA, TXS, TYA
} ins_name_type;
ins_name_type ins_name_unreg;   // Use this during LOAD_INS (combinatorial)
ins_name_type ins_name;         // Use this after LOAD_INS (registered)

typedef enum bit [3:0] {
    ADDR_MODE_UNDEFINED,
    ACC, ABS, ABS_X, ABS_Y, IMM, IMPL, IND, X_IND, IND_Y, REL, ZPG, ZPG_X, ZPG_Y
} addr_mode_type;
addr_mode_type addr_mode_unreg; // Use this during LOAD_INS (combinatorial)
addr_mode_type addr_mode;       // Use this after LOAD_INS (registered)

logic write_only_unreg, write_only;
logic write_result_unreg, write_result;
logic impl_no_maccess_unreg, impl_no_maccess;

// FSM States
enum bit[5:0] {
    _UNUSED_ZERO_,  // Prevent the simulator from not triggering combinatorial processes
    RST0, RST1,
    T0,
    LOP1, LOP2,
    TXFR,
    EXECM,
    STORM,
    IND3, IND4,
    JSR2, JSR3, JSR4,
    EXEC1,
    REL1, REL2, RELD3, RELU3,
    INDX1, INDX2, INDX3,
    INDY1, INDY2, INDY3,
    PUSH1, PUSH2,
    PUL1, PUL2,     // PULL1 may be a reserved word(?), avoid it and use PULx instead
    RTS2, RTS3, RTS4,
    BRK1, BRK2, BRK3, BRK4, BRK5, BRK6,
    INT1, INT2, INT3, INT4, INT5,
    RTI1, RTI2, RTI3, RTI4
} state;

//******************************************************************************
// FSM Next State
//******************************************************************************
always @(posedge clk) begin
    if (reset) state <= RST0;
    else if (rdy) case (state)
        RST0: state <= RST1;

        RST1, RELD3, RELU3, EXEC1,
        JSR4, IND4, TXFR, STORM,
        PUSH2, PUL2, RTS4,
        BRK6, INT5, RTI4:             state <= T0;

        T0: if (int_pending)          state <= INT1;
        else case (addr_mode_unreg)
            REL:                      state <= REL1;
            ACC, ADDR_MODE_UNDEFINED: state <= EXEC1;
            ABS, ABS_X, ABS_Y,
            ZPG, ZPG_X, ZPG_Y,
            IND:                      state <= LOP1;
            IMM:                      state <= EXECM;
            X_IND:                    state <= INDX1;
            IND_Y:                    state <= INDY1;
            IMPL:
                if (impl_no_maccess_unreg) state <= EXEC1;
                else case (ins_name_unreg)
                    PLA, PLP, RTS:    state <= PUL1;
                    PHA, PHP:         state <= PUSH1;
                    BRK:              state <= BRK1;
                    RTI:              state <= RTI1;
                    default:          state <= RST0;
                endcase
            default:                  state <= RST0;
        endcase

        LOP1: case (addr_mode)
            ZPG, ZPG_X, ZPG_Y:
                if      (write_only)      state <= TXFR;
                else                      state <= EXECM;
            ABS, ABS_X, ABS_Y, IND:
                if (ins_name == JSR)      state <= JSR2; // assert ABS
                else                      state <= LOP2;
            default:                      state <= RST0;
        endcase

        LOP2: case (addr_mode)
            ABS, ABS_X, ABS_Y:
                if      (ins_name == JMP) state <= T0;
                else if (write_only)      state <= TXFR;
                else                      state <= EXECM;
            IND:                          state <= IND3;
            default:                      state <= RST0;
        endcase

        EXECM:
            if (write_result) state <= STORM;
            else              state <= T0;

        REL1:
            if ((ins_name == BCS && c) || (ins_name == BCC && !c) ||
                (ins_name == BEQ && z) || (ins_name == BNE && !z) ||
                (ins_name == BMI && n) || (ins_name == BPL && !n) ||
                (ins_name == BVS && v) || (ins_name == BVC && !v))   state <= REL2; else state <= T0;

        REL2: state <= (iw[7] ^ alu_c) ? (alu_c ? RELU3 : RELD3) : T0;

        PUL1: state <= (ins_name == RTS) ? RTS2 : PUL2;
        RTS2: state <= RTS3;
        RTS3: state <= RTS4;

        PUSH1: state <= PUSH2;

        JSR2: state <= JSR3;
        JSR3: state <= JSR4;

        INDX1: state <= INDX2;
        INDX2: state <= INDX3;

        INDY1: state <= INDY2;
        INDY2: state <= INDY3;

        IND3: state <= IND4;

        INDX3, INDY3:
            if (write_only) state <= TXFR;
            else            state <= EXECM;

        BRK1: state <= BRK2;
        BRK2: state <= BRK3;
        BRK3: state <= BRK4;
        BRK4: state <= BRK5;
        BRK5: state <= BRK6;

        INT1: state <= INT2;
        INT2: state <= INT3;
        INT3: state <= INT4;
        INT4: state <= INT5;

        RTI1: state <= RTI2;
        RTI2: state <= RTI3;
        RTI3: state <= RTI4;

        default: state <= RST0;
    endcase
end

//******************************************************************************
// Address / r_w / data_out
//******************************************************************************
always @(*) begin
    case (state)
        RST0: begin
            r_w = 1; addr = 16'hFFFC; data_out = 8'bX;
        end

        RST1: begin
            r_w = 1; addr = 16'hFFFD; data_out = 8'bX;
        end

        T0, LOP1, LOP2, REL1, INDX1, INDY1, IND3, IND4, JSR4: begin
            r_w = 1; addr = pc; data_out = 8'bX;
        end

        EXECM, INDX2, INDX3, INDY2, INDY3: begin
            r_w = 1; addr = ip; data_out = 8'bX;
        end

        STORM: begin
            r_w = 0; addr = ip; data_out = iw;
        end

        PUL2, RTS2, RTS4, RTI2, RTI3, RTI4: begin
            r_w = 1; addr = {8'h01, s}; data_out = 8'bX;
        end

        PUSH1: begin
            r_w = 0; addr = {8'h01, s}; data_out = alu_q;
        end

        JSR2, BRK2, INT1: begin
            r_w = 0; addr = {8'h01, s}; data_out = pc[15:8];
        end

        JSR3, BRK3, INT2: begin
            r_w = 0; addr = {8'h01, s}; data_out = pc[7:0];
        end

        BRK4: begin
            r_w = 0; addr = {8'h01, s}; data_out = {n, v, 1'b1, 1'b1, d, i, z, c};
        end

        INT3: begin
            r_w = 0; addr = {8'h01, s}; data_out = {n, v, 1'b1, 1'b0, d, i, z, c};
        end

        BRK5: begin
            r_w = 1; addr = 16'hFFFE; data_out = 8'bX;
        end

        BRK6: begin
            r_w = 1; addr = 16'hFFFF; data_out = 8'bX;
        end

        INT4: begin
            r_w = 1; addr = (nmi_pending ? 16'hFFFA : 16'hFFFE); data_out = 8'bX;
        end

        INT5: begin
            r_w = 1; addr = (nmi_pending ? 16'hFFFB : 16'hFFFF); data_out = 8'bX;
        end

        TXFR: begin
            case (ins_name)
                STA: begin
                    r_w = 0; addr = ip; data_out = a;
                end

                STX: begin
                    r_w = 0; addr = ip; data_out = x;
                end

                STY: begin
                    r_w = 0; addr = ip; data_out = y;
                end

                default: begin
                    r_w = 1; addr = 16'bXXXX; data_out = 8'bX;
                end
            endcase
        end

        default: begin
            r_w = 1; addr = 16'bXXXX; data_out = 8'bX;
        end
    endcase
end

//******************************************************************************
// PC
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        RST0, BRK5, INT4, RTI3:      pc[7:0] <= data_in;
        RST1, BRK6, INT5, RTI4:      pc[15:8] <= data_in;
        T0: if (!int_pending)        pc <= pc + 1;
        REL1, LOP1, IND3,
        INDX1, INDY1, BRK1:          pc <= pc + 1;
        REL2:                        pc[7:0] <= alu_q;
        RELD3, RELU3:                pc[15:8] <= alu_q;
        JSR4, IND4:                  pc <= {data_in, ip[7:0]};
        LOP2:
            if (ins_name == JMP)     pc <= {data_in, ip[7:0]};
            else                     pc <= pc + 1;
        EXECM:
            if (addr_mode == IMM)    pc <= pc + 1;
        RTS4:                        pc <= {alu_q, iw};
    endcase
end

//******************************************************************************
// ALU Driver
//******************************************************************************
always @(*) begin

alu_i1 = 8'bXXXXXXXX;
alu_i2 = 8'bXXXXXXXX;
alu_i3 = 1'bX;
alu_op = OP_DONTCARE;

case (state)
    EXECM: begin
        case (ins_name)
            ADC: begin
                alu_op = d ? OP_ADD_BCD : OP_ADD;
                alu_i1 = a;
                alu_i2 = data_in;
                alu_i3 = c;
            end

            AND, BIT: begin
                alu_op = OP_AND;
                alu_i1 = a;
                alu_i2 = data_in;
            end

            CMP: begin
                alu_op = OP_SUB;
                alu_i1 = a;
                alu_i2 = data_in;
                alu_i3 = 1'b1;
            end

            CPX: begin
                alu_op = OP_SUB;
                alu_i1 = x;
                alu_i2 = data_in;
                alu_i3 = 1'b1;
            end

            CPY: begin
                alu_op = OP_SUB;
                alu_i1 = y;
                alu_i2 = data_in;
                alu_i3 = 1'b1;
            end

            DEC: begin
                alu_op = OP_SUB;
                alu_i1 = data_in;
                alu_i2 = 8'b0;
                alu_i3 = 1'b0;
            end

            DEX: begin
                alu_op = OP_SUB;
                alu_i1 = x;
                alu_i2 = 8'b0;
                alu_i3 = 1'b0;
            end

            DEY: begin
                alu_op = OP_SUB;
                alu_i1 = y;
                alu_i2 = 8'b0;
                alu_i3 = 1'b0;
            end

            INC: begin
                alu_op = OP_ADD;
                alu_i1 = data_in;
                alu_i2 = 8'b0;
                alu_i3 = 1'b1;
            end

            INX: begin
                alu_op = OP_ADD;
                alu_i1 = x;
                alu_i2 = 8'b0;
                alu_i3 = 1'b1;
            end

            INY: begin
                alu_op = OP_ADD;
                alu_i1 = y;
                alu_i2 = 8'b0;
                alu_i3 = 1'b1;
            end

            LDA, LDX, LDY: begin
                alu_op = OP_IDT;
                alu_i1 = data_in;
            end

            ASL: begin
                alu_op = OP_ROL;
                alu_i1 = data_in;
                alu_i3 = 1'b0;
            end

            LSR: begin
                alu_op = OP_ROR;
                alu_i1 = data_in;
                alu_i3 = 1'b0;
            end

            ROL: begin
                alu_op = OP_ROL;
                alu_i1 = data_in;
                alu_i3 = c;
            end

            ROR: begin
                alu_op = OP_ROR;
                alu_i1 = data_in;
                alu_i3 = c;
            end

            EOR: begin
                alu_op = OP_XOR;
                alu_i1 = a;
                alu_i2 = data_in;
            end

            ORA: begin
                alu_op = OP_OR;
                alu_i1 = a;
                alu_i2 = data_in;
            end

            STX: begin
                alu_op = OP_IDT;
                alu_i1 = x;
            end

            STY: begin
                alu_op = OP_IDT;
                alu_i1 = y;
            end

            SBC: begin
                alu_op = d ? OP_SUB_BCD : OP_SUB;
                alu_i1 = a;
                alu_i2 = data_in;
                alu_i3 = c;
            end
        endcase
    end

    EXEC1: begin
        case (ins_name)
            ASL: begin
                alu_op = OP_ROL;
                alu_i1 = a;
                alu_i3 = 1'b0;
            end

            LSR: begin
                alu_op = OP_ROR;
                alu_i1 = a;
                alu_i3 = 1'b0;
            end

            ROL: begin
                alu_op = OP_ROL;
                alu_i1 = a;
                alu_i3 = c;
            end

            ROR: begin
                alu_op = OP_ROR;
                alu_i1 = a;
                alu_i3 = c;
            end

            TXA, TXS: begin
                alu_op = OP_IDT;
                alu_i1 = x;
            end

            TYA: begin
                alu_op = OP_IDT;
                alu_i1 = y;
            end

            TSX: begin
                alu_op = OP_IDT;
                alu_i1 = s;
            end

            TAY, TAX: begin
                alu_op = OP_IDT;
                alu_i1 = a;
            end

            DEX: begin
                alu_op = OP_SUB;
                alu_i1 = x;
                alu_i2 = 0;
                alu_i3 = 1'b0;
            end

            DEY: begin
                alu_op = OP_SUB;
                alu_i1 = y;
                alu_i2 = 0;
                alu_i3 = 1'b0;
            end

            INX: begin
                alu_op = OP_ADD;
                alu_i1 = x;
                alu_i2 = 0;
                alu_i3 = 1'b1;
            end

            INY: begin
                alu_op = OP_ADD;
                alu_i1 = y;
                alu_i2 = 0;
                alu_i3 = 1'b1;
            end
        endcase
    end

    REL2: begin
        alu_op = OP_ADD;
        alu_i1 = pc[7:0];
        alu_i2 = iw;
        alu_i3 = 1'b0;
    end

    RELU3: begin
        alu_op = OP_ADD;
        alu_i1 = pc[15:8];
        alu_i2 = 8'b0;
        alu_i3 = 1'b1;
    end

    RELD3: begin
        alu_op = OP_SUB;
        alu_i1 = pc[15:8];
        alu_i2 = 8'b0;
        alu_i3 = 1'b0;
    end

    LOP1: begin
        case (addr_mode)
            ABS, ZPG, IND: begin
                alu_op = OP_IDT;
                alu_i1 = data_in;
            end

            ABS_X, ZPG_X: begin
                alu_op = OP_ADD;
                alu_i1 = x;
                alu_i2 = data_in;
                alu_i3 = 1'b0;
            end

            ABS_Y, ZPG_Y: begin
                alu_op = OP_ADD;
                alu_i1 = y;
                alu_i2 = data_in;
                alu_i3 = 1'b0;
            end
        endcase
    end

    LOP2: begin
        case (addr_mode)
            ABS: begin
                alu_op = OP_IDT;
                alu_i1 = data_in;
            end

            ABS_X, ABS_Y: begin
                alu_op = OP_ADD;
                alu_i1 = data_in;
                alu_i2 = 8'b0;
                alu_i3 = ifl;
            end
        endcase
    end

    INDX1: begin
        alu_op = OP_ADD;
        alu_i1 = x;
        alu_i2 = data_in;
        alu_i3 = 1'b0;
    end

    INDX2: begin
        alu_op = OP_ADD;
        alu_i1 = ip[7:0];
        alu_i2 = 8'b0;
        alu_i3 = 1;
    end

    INDY1, RTS2: begin
        alu_op = OP_ADD;
        alu_i1 = data_in;
        alu_i2 = 8'b0;
        alu_i3 = 1'b1;
    end

    INDY2: begin
        alu_op = OP_ADD;
        alu_i1 = y;
        alu_i2 = data_in;
        alu_i3 = 1'b0;
    end

    INDY3, RTS4: begin
        alu_op = OP_ADD;
        alu_i1 = data_in;
        alu_i2 = 8'b0;
        alu_i3 = ifl;
    end

    PUSH1: begin
        alu_op = OP_IDT;
        alu_i1 = (ins_name == PHA) ? a
               : (ins_name == PHP) ? {n, v, 1'b1, 1'b1, d, i, z, c}
               : 16'haa;
    end

    PUSH2, JSR2, JSR3, BRK2, BRK3, BRK4, INT1, INT2, INT3: begin
        alu_op = OP_SUB;
        alu_i1 = s;
        alu_i2 = 8'b0;
        alu_i3 = 1'b0;
    end

    PUL1, RTS3, RTI1, RTI2, RTI3: begin
        alu_op = OP_ADD;
        alu_i1 = s;
        alu_i2 = 8'b0;
        alu_i3 = 1'b1;
    end

    PUL2: case (ins_name)
        PLA: begin
            alu_op = OP_IDT;
            alu_i1 = data_in;
        end
    endcase
endcase
end

//******************************************************************************
// Internal registers
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        T0:                 ip <= pc + 1;
        LOP1, INDX1, INDX2: ip <= {8'b0, alu_q};
        INDX3:              ip <= {data_in, iw};
        INDY1:              ip <= {8'b0, data_in};
        INDY2:              ip <= {8'b0, iw};
        INDY3:              ip <= {alu_q, iw};
        LOP2:               ip[15:8] <= alu_q;
        IND3:               ip[7:0] <= data_in;
    endcase
    if (reset) ip <= 16'bX;
end

always @(posedge clk) begin
    if (rdy) case (state)
        EXECM, INDY1, INDY2, RTS2: iw <= alu_q;
        REL1, INDX2: iw <= data_in;
    endcase
    if (reset) iw <= 8'bX;
end

always @(posedge clk) begin
    if (rdy) case (state)
        LOP1, INDY2, RTS2: ifl <= alu_c;
    endcase
    if (reset) ifl <= 1'bX;
end

//******************************************************************************
// Accumulator
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        EXEC1: case (ins_name)
            LSR, ASL, ROL, ROR,
            TXA, TYA:
                a <= alu_q;
        endcase

        EXECM: case (ins_name)
            ADC, SBC, LDA,
            ORA, AND, EOR:
                a <= alu_q;
        endcase

        PUL2: case (ins_name)
            PLA: a <= alu_q;
        endcase
    endcase
    if (reset) a <= 8'bX;
end

//******************************************************************************
// X/Y Index
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        EXEC1, EXECM: begin
            case (ins_name)
                TAX, TSX, LDX, DEX, INX: x <= alu_q;
            endcase
        end
    endcase
    if (reset) x <= 8'bX;
end

always @(posedge clk) begin
    if (rdy) case (state)
        EXEC1, EXECM: begin
            case (ins_name)
                TAY, LDY, DEY, INY: y <= alu_q;
            endcase
        end
    endcase
    if (reset) y <= 8'bX;
end

//******************************************************************************
// Stack Pointer
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        EXEC1: begin
            case (ins_name)
                TXS: s <= alu_q;
            endcase
        end

        PUSH2, PUL1,
        JSR2, JSR3,
        RTS3,
        BRK2, BRK3, BRK4,
        INT1, INT2, INT3,
        RTI1, RTI2, RTI3: s <= alu_q;
    endcase
    if (reset) s <= 8'bX;
end

//******************************************************************************
// Processor Status
//******************************************************************************
always @(posedge clk) begin
    if (rdy) case (state)
        T0: begin
            if (so_pending) v <= 1'b1;
        end

        EXEC1, EXECM: case (ins_name)
            SEC: c <= 1'b1;
            CLC: c <= 1'b0;
            SED: d <= 1'b1;
            CLD: d <= 1'b0;
            SEI: i <= 1'b1;
            CLI: i <= 1'b0;
            CLV: v <= 1'b0;

            ADC, SBC: begin
                n <= alu_n;
                z <= alu_z;
                c <= alu_c;
                v <= alu_v;
            end

            AND, DEC, DEX, DEY,
            EOR, INC, INX, INY,
            LDA, LDX, LDY, ORA,
            TAX, TAY, TSX, TXA,
            TYA: begin
                n <= alu_n;
                z <= alu_z;
            end

            BIT: begin
                n <= data_in[7];
                z <= alu_z;
                v <= data_in[6];
            end

            CMP, CPX, CPY, ASL,
            LSR, ROL, ROR: begin
                n <= alu_n;
                z <= alu_z;
                c <= alu_c;
            end
        endcase

        PUL2: case (ins_name)
            PLP: {n, v, d, i, z, c} <= {data_in[7:6], data_in[3:0]};
            PLA: begin
                n <= alu_n;
                z <= alu_z;
            end
        endcase

        RTI2: {n, v, d, i, z, c} <= {data_in[7:6], data_in[3:0]};

        BRK4, INT3: i <= 1'b1;
    endcase

    if (reset) begin
        n <= 1'bX;
        z <= 1'bX;
        c <= 1'bX;
        v <= 1'bX;
        i <= 1'b1;
        d <= 1'bX;
    end
end

//******************************************************************************
// ALU
//******************************************************************************
wire [7:0] alu_i2_comp = ~alu_i2;
wire alu_i3_comp = ~alu_i3;

always @(*) begin
    case (alu_op)
        OP_ADD: begin
            {alu_c, alu_q} = alu_i1 + alu_i2 + alu_i3;
            alu_n = alu_q[7];
            alu_z = (alu_q == 8'b0);
            alu_v = (alu_i1[7] && alu_i2[7] && !alu_q[7])
                    || (!alu_i1[7] && !alu_i2[7] && alu_q[7]);
        end

        OP_ADD_BCD: begin
            logic sum0_c, sum0_nc, sum06_c;
            logic [7:0] sum0_q, sum06_q;
            
            {sum0_c, sum0_q} = alu_i1 + alu_i2 + alu_i3;
            sum0_nc = alu_i1[4] ^ alu_i2[4] ^ sum0_q[4];  // Nibble Carry
            
            {sum06_c, sum06_q} = alu_i1 + alu_i2 + alu_i3 + 8'h06;
            
            if ({sum0_nc, sum0_q[3:0]} > 5'd9)
                if ({sum06_c, sum06_q[7:4]} > 5'd9)
                    {alu_c, alu_q} = alu_i1 + alu_i2 + alu_i3 + 8'h66;
                else
                    {alu_c, alu_q} = {sum06_c, sum06_q};
            else
                if ({sum0_c, sum0_q[7:4]} > 5'd9)
                    {alu_c, alu_q} = alu_i1 + alu_i2 + alu_i3 + 8'h60;
                else
                    {alu_c, alu_q} = {sum0_c, sum0_q};

            alu_n = 1'bX;
            alu_z = 1'bX;;
            alu_v = 1'bX;
        end

        OP_SUB: begin
            {alu_c, alu_q} = alu_i1 + alu_i2_comp + alu_i3;
            alu_n = alu_q[7];
            alu_z = (alu_q == 8'b0);
            alu_v = (alu_i1[7] && alu_i2_comp[7] && !alu_q[7])
                    || (!alu_i1[7] && !alu_i2_comp[7] && alu_q[7]);
        end

        OP_SUB_BCD: begin
            logic bin_c, bin_nc;
            logic [7:0] bin_q;
            {bin_c, bin_q} = alu_i1 - alu_i2 - alu_i3_comp;
            bin_nc = alu_i1[4] ^ alu_i2[4] ^ bin_q[4];  // Nibble Carry
            if ({bin_nc, bin_q[3:0]} > 4'd9)
                if ({bin_c, bin_q[7:4]} > 9)
                    {alu_c, alu_q} = alu_i1 - alu_i2 - alu_i3_comp - 8'h66;
                else
                    {alu_c, alu_q} = alu_i1 - alu_i2 - alu_i3_comp - 8'h06;
            else
                if ({bin_c, bin_q[7:4]} > 9)
                    {alu_c, alu_q} = alu_i1 - alu_i2 - alu_i3_comp - 8'h60;
                else
                    {alu_c, alu_q} = alu_i1 - alu_i2 - alu_i3_comp;
            alu_c = ~alu_c;
            alu_n = 1'bX;
            alu_z = 1'bX;
            alu_v = 1'bX;
        end

        OP_ROR: begin
            alu_q = {alu_i3, alu_i1[7:1]};
            alu_n = alu_i3;
            alu_c = alu_i1[0];
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        OP_ROL: begin
            alu_q = {alu_i1[6:0], alu_i3};
            alu_n = alu_i1[6];
            alu_c = alu_i1[7];
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        OP_IDT: begin
            alu_q = alu_i1;
            alu_n = alu_q[7];
            alu_c = 1'bX;
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        OP_AND: begin
            alu_q = alu_i1 & alu_i2;
            alu_n = alu_q[7];
            alu_c = 1'bX;
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        OP_OR: begin
            alu_q = alu_i1 | alu_i2;
            alu_n = alu_q[7];
            alu_c = 1'bX;
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        OP_XOR: begin
            alu_q = alu_i1 ^ alu_i2;
            alu_n = alu_q[7];
            alu_c = 1'bX;
            alu_z = (alu_q == 8'b0);
            alu_v = 1'bX;
        end

        default: begin
            alu_q = 8'bX;
            alu_n = 1'bX;
            alu_c = 1'bX;
            alu_z = 1'bX;
            alu_v = 1'bX;
        end
    endcase
end

//******************************************************************************
// NMI Edge detector
//******************************************************************************
logic nmi_prev;
always @(posedge clk) begin
    if (reset) nmi_prev <= 1'b0;
    else       nmi_prev <= nmi;
end

always @(posedge clk) begin
    if (reset)                 nmi_pending <= 1'b0;
    else if (state == INT5)    nmi_pending <= 1'b0;
    else if (nmi && !nmi_prev) nmi_pending <= 1'b1; // Posedge of nmi
end

assign int_pending = ((irq && !i) || nmi_pending);

//******************************************************************************
// SO Edge detector
//******************************************************************************
logic so_prev;
always @(posedge clk) begin
    if (reset) so_prev <= 1'b0;
    else       so_prev <= so;
end

always @(posedge clk) begin
    if (reset)               so_pending <= 1'b0;
    else if (so && !so_prev) so_pending <= 1'b1; // Posedge of so
    else if (state == T0)    so_pending <= 1'b0;
end

//******************************************************************************
// Sync
//******************************************************************************
assign sync = (state == T0);

//******************************************************************************
// Debug
//******************************************************************************
assign invalid_op = (state == EXEC1 && ins_name == INS_UNDEFINED);

//******************************************************************************
// Instruction decoder
//******************************************************************************
always @(posedge clk) begin
    if (rdy && state == T0) begin
        ins_name <= ins_name_unreg;
        addr_mode <= addr_mode_unreg;
        write_only <= write_only_unreg;
        write_result <= write_result_unreg;
        impl_no_maccess <= impl_no_maccess_unreg;
    end
end

always @(*) begin
    case (data_in)
        8'h06, 8'h0A, 8'h0E, 8'h16, 8'h1E: ins_name_unreg = ASL;
        8'h88: ins_name_unreg = DEY;
        8'h86, 8'h8E, 8'h96: ins_name_unreg = STX;
        8'hE0, 8'hE4, 8'hEC: ins_name_unreg = CPX;
        8'hF8: ins_name_unreg = SED;
        8'h20: ins_name_unreg = JSR;
        8'h78: ins_name_unreg = SEI;
        8'h98: ins_name_unreg = TYA;
        8'h61, 8'h65, 8'h69, 8'h6D, 8'h71, 8'h75, 8'h79, 8'h7D: ins_name_unreg = ADC;
        8'hC1, 8'hC5, 8'hC9, 8'hCD, 8'hD1, 8'hD5, 8'hD9, 8'hDD: ins_name_unreg = CMP;
        8'hD8: ins_name_unreg = CLD;
        8'h9A: ins_name_unreg = TXS;
        8'h66, 8'h6A, 8'h6E, 8'h76, 8'h7E: ins_name_unreg = ROR;
        8'h10: ins_name_unreg = BPL;
        8'hA0, 8'hA4, 8'hAC, 8'hB4, 8'hBC: ins_name_unreg = LDY;
        8'h58: ins_name_unreg = CLI;
        8'h8A: ins_name_unreg = TXA;
        8'h60: ins_name_unreg = RTS;
        8'hB0: ins_name_unreg = BCS;
        8'hE6, 8'hEE, 8'hF6, 8'hFE: ins_name_unreg = INC;
        8'hD0: ins_name_unreg = BNE;
        8'h18: ins_name_unreg = CLC;
        8'hA2, 8'hA6, 8'hAE, 8'hB6, 8'hBE: ins_name_unreg = LDX;
        8'h28: ins_name_unreg = PLP;
        8'h81, 8'h85, 8'h8D, 8'h91, 8'h95, 8'h99, 8'h9D: ins_name_unreg = STA;
        8'h50: ins_name_unreg = BVC;
        8'h24, 8'h2C: ins_name_unreg = BIT;
        8'hC0, 8'hC4, 8'hCC: ins_name_unreg = CPY;
        8'hB8: ins_name_unreg = CLV;
        8'hBA: ins_name_unreg = TSX;
        8'h26, 8'h2A, 8'h2E, 8'h36, 8'h3E: ins_name_unreg = ROL;
        8'hC6, 8'hCE, 8'hD6, 8'hDE: ins_name_unreg = DEC;
        8'hF0: ins_name_unreg = BEQ;
        8'hCA: ins_name_unreg = DEX;
        8'hA8: ins_name_unreg = TAY;
        8'h38: ins_name_unreg = SEC;
        8'h01, 8'h05, 8'h09, 8'h0D, 8'h11, 8'h15, 8'h19, 8'h1D: ins_name_unreg = ORA;
        8'h00: ins_name_unreg = BRK;
        8'hC8: ins_name_unreg = INY;
        8'h68: ins_name_unreg = PLA;
        8'h90: ins_name_unreg = BCC;
        8'h46, 8'h4A, 8'h4E, 8'h56, 8'h5E: ins_name_unreg = LSR;
        8'h70: ins_name_unreg = BVS;
        8'hAA: ins_name_unreg = TAX;
        8'h48: ins_name_unreg = PHA;
        8'h30: ins_name_unreg = BMI;
        8'h40: ins_name_unreg = RTI;
        8'hE8: ins_name_unreg = INX;
        8'hE1, 8'hE5, 8'hE9, 8'hED, 8'hF1, 8'hF5, 8'hF9, 8'hFD: ins_name_unreg = SBC;
        8'h41, 8'h45, 8'h49, 8'h4D, 8'h51, 8'h55, 8'h59, 8'h5D: ins_name_unreg = EOR;
        8'h4C, 8'h6C: ins_name_unreg = JMP;
        8'h21, 8'h25, 8'h29, 8'h2D, 8'h31, 8'h35, 8'h39, 8'h3D: ins_name_unreg = AND;
        8'hA1, 8'hA5, 8'hA9, 8'hAD, 8'hB1, 8'hB5, 8'hB9, 8'hBD: ins_name_unreg = LDA;
        8'h84, 8'h8C, 8'h94: ins_name_unreg = STY;
        8'h08: ins_name_unreg = PHP;
        8'hEA: ins_name_unreg = NOP;
        default: ins_name_unreg = INS_UNDEFINED;
    endcase
end

always @(*) begin
    case (data_in)
        8'h11, 8'h31, 8'h51, 8'h71, 8'h91, 8'hB1, 8'hD1, 8'hF1:
            addr_mode_unreg = IND_Y;
        8'h09, 8'h29, 8'h49, 8'h69, 8'hA0, 8'hA2, 8'hA9, 8'hC0, 8'hC9, 8'hE0, 8'hE9:
            addr_mode_unreg = IMM;
        8'h10, 8'h30, 8'h50, 8'h70, 8'h90, 8'hB0, 8'hD0, 8'hF0:
            addr_mode_unreg = REL;
        8'h05, 8'h06, 8'h24, 8'h25, 8'h26, 8'h45, 8'h46, 8'h65, 8'h66, 8'h84, 8'h85, 8'h86, 8'hA4, 8'hA5, 8'hA6, 8'hC4, 8'hC5, 8'hC6, 8'hE4, 8'hE5, 8'hE6:
            addr_mode_unreg = ZPG;
        8'h00, 8'h08, 8'h18, 8'h28, 8'h38, 8'h40, 8'h48, 8'h58, 8'h60, 8'h68, 8'h78, 8'h88, 8'h8A, 8'h98, 8'h9A, 8'hA8, 8'hAA, 8'hB8, 8'hBA, 8'hC8, 8'hCA, 8'hD8, 8'hE8, 8'hEA, 8'hF8:
            addr_mode_unreg = IMPL;
        8'h96, 8'hB6:
            addr_mode_unreg = ZPG_Y;
        8'h0A, 8'h2A, 8'h4A, 8'h6A:
            addr_mode_unreg = ACC;
        8'h19, 8'h39, 8'h59, 8'h79, 8'h99, 8'hB9, 8'hBE, 8'hD9, 8'hF9:
            addr_mode_unreg = ABS_Y;
        8'h01, 8'h21, 8'h41, 8'h61, 8'h81, 8'hA1, 8'hC1, 8'hE1:
            addr_mode_unreg = X_IND;
        8'h1D, 8'h1E, 8'h3D, 8'h3E, 8'h5D, 8'h5E, 8'h7D, 8'h7E, 8'h9D, 8'hBC, 8'hBD, 8'hDD, 8'hDE, 8'hFD, 8'hFE:
            addr_mode_unreg = ABS_X;
        8'h0D, 8'h0E, 8'h20, 8'h2C, 8'h2D, 8'h2E, 8'h4C, 8'h4D, 8'h4E, 8'h6D, 8'h6E, 8'h8C, 8'h8D, 8'h8E, 8'hAC, 8'hAD, 8'hAE, 8'hCC, 8'hCD, 8'hCE, 8'hEC, 8'hED, 8'hEE:
            addr_mode_unreg = ABS;
        8'h15, 8'h16, 8'h35, 8'h36, 8'h55, 8'h56, 8'h75, 8'h76, 8'h94, 8'h95, 8'hB4, 8'hB5, 8'hD5, 8'hD6, 8'hF5, 8'hF6:
            addr_mode_unreg = ZPG_X;
        8'h6C:
            addr_mode_unreg = IND;
        default: addr_mode_unreg = ADDR_MODE_UNDEFINED;
    endcase
end

always @(*) begin
    case (data_in)
        8'h81, 8'h84, 8'h85, 8'h86, 8'h8C, 8'h8D, 8'h8E, 8'h91, 8'h94, 8'h95, 8'h96, 8'h99, 8'h9D:
            write_only_unreg = 1'b1;
        default:
            write_only_unreg = 1'b0;
    endcase
end

always @(*) begin
    case (data_in)
        8'h06, 8'h0E, 8'h16, 8'h1E, 8'h26, 8'h2E, 8'h36, 8'h3E, 8'h46, 8'h4E, 8'h56, 8'h5E, 8'h66, 8'h6E, 8'h76, 8'h7E, 8'hC6, 8'hCE, 8'hD6, 8'hDE, 8'hE6, 8'hEE, 8'hF6, 8'hFE:
            write_result_unreg = 1'b1;
        default:
            write_result_unreg = 1'b0;
    endcase
end

always @(*) begin
    case (data_in)
        8'h18, 8'h38, 8'h58, 8'h78, 8'h88, 8'h8A, 8'h98, 8'h9A, 8'hA8, 8'hAA, 8'hB8, 8'hBA, 8'hC8, 8'hCA, 8'hD8, 8'hE8, 8'hEA, 8'hF8:
            impl_no_maccess_unreg = 1'b1;
        default:
            impl_no_maccess_unreg = 1'b0;
    endcase
end

endmodule
