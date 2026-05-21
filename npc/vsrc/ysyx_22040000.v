// =============================================================================
// ysyx_22040000  --  D6 SoC top wrapper
// =============================================================================
// Multi-cycle minirv CPU that speaks SimpleBus (ysyxSoC `MemBridge` protocol).
//
// Pin names / widths match `ysyxSoC/ready-to-run/D-stage/cpu-interface.md`
// exactly. The module instantiates the same IDU / EXU / RegFile / CSR / WBU
// modules used by the DPI harness, but replaces the combinational IFU / LSU
// with bus state machines so each fetch / load / store waits for `respValid`.
//
// State machine summary (single FSM that sequences every instruction):
//   S_IF : drive io_ifu_reqValid=1, wait for io_ifu_respValid, latch instruction.
//   S_EX : decode/execute combinationally. For non-load/store -> go S_WB
//          directly. For load/store -> drive io_lsu_reqValid=1, wait for
//          io_lsu_respValid, latch rdata (load) / observe bready (store).
//   S_WB : commit: write regfile (if reg_wen), write CSR (if csr_we), update PC,
//          then go back to S_IF.
//
// The wrapper intentionally uses a single posedge process for FSM updates and
// only asserts `reg_wen` / `csr_we` for one cycle (in S_WB). This avoids
// duplicate writes that would otherwise happen because IDU is purely
// combinational and fires every cycle the instruction is held.
//
// Reset vector: 0x30000000 (Flash base on the SoC).
// =============================================================================
`ifndef SOC_PC_RESET_VEC
  `define SOC_PC_RESET_VEC 32'h3000_0000
`endif

module ysyx_22040000(
  input         clock,
  input         reset,
  // ---- IFU master ----------------------------------------------------------
  output        io_ifu_reqValid,
  output [31:0] io_ifu_addr,
  input         io_ifu_respValid,
  input  [31:0] io_ifu_rdata,
  // ---- LSU master ----------------------------------------------------------
  output        io_lsu_reqValid,
  output [31:0] io_lsu_addr,
  output [1:0]  io_lsu_size,
  output        io_lsu_wen,
  output [31:0] io_lsu_wdata,
  output [3:0]  io_lsu_wmask,
  input         io_lsu_respValid,
  input  [31:0] io_lsu_rdata
);
  // ---- FSM ------------------------------------------------------------------
  localparam S_IF = 2'd0;
  localparam S_EX = 2'd1;
  localparam S_LS = 2'd2;
  localparam S_WB = 2'd3;

  reg [1:0]  state, next_state;

  // ---- PC and latched instruction ------------------------------------------
  reg  [31:0] pc;
  reg  [31:0] inst_r;
  wire [31:0] inst = (state == S_IF) ? io_ifu_rdata : inst_r;

  // ---- Decode (combinational, on `inst`) -----------------------------------
  wire [3:0]  rs1_a, rs2_a, rd_a;
  wire [31:0] imm;
  wire        alu_src;
  wire [1:0]  alu_op;
  wire        mem_re, mem_we;
  wire [2:0]  funct3;
  wire [1:0]  wb_sel;
  wire        reg_wen_dec;
  wire        is_jalr;
  wire        is_ebreak;
  wire        is_csr;
  wire [11:0] csr_addr;
  wire [4:0]  csr_uimm;
  wire        csr_use_uimm;
  wire        csr_re;
  wire        csr_we_dec;
  wire [1:0]  csr_op;

  IDU u_idu (
    .inst(inst),
    .rs1(rs1_a), .rs2(rs2_a), .rd(rd_a),
    .imm(imm), .alu_src(alu_src), .alu_op(alu_op),
    .mem_re(mem_re), .mem_we(mem_we), .funct3(funct3),
    .wb_sel(wb_sel), .reg_wen(reg_wen_dec),
    .is_jalr(is_jalr), .is_ebreak(is_ebreak),
    .is_csr(is_csr), .csr_addr(csr_addr),
    .csr_uimm(csr_uimm), .csr_use_uimm(csr_use_uimm),
    .csr_re(csr_re), .csr_we(csr_we_dec), .csr_op(csr_op)
  );

  // ---- Register file --------------------------------------------------------
  wire [31:0] rs1_val, rs2_val;
  wire [31:0] wb_data;
  // Only commit the register write in S_WB to avoid double-writes.
  wire        reg_wen_wb = reg_wen_dec & (state == S_WB) & ~is_ebreak;

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clock),
    .raddr1(rs1_a), .rdata1(rs1_val),
    .raddr2(rs2_a), .rdata2(rs2_val),
    .wdata(wb_data), .waddr(rd_a), .wen(reg_wen_wb)
  );

  // ---- EXU ------------------------------------------------------------------
  wire [31:0] alu_result;
  EXU u_exu (
    .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm),
    .alu_src(alu_src), .alu_op(alu_op),
    .alu_result(alu_result)
  );

  // ---- LSU: build aligned store word + wmask --------------------------------
  // Load data path will choose between bus rdata (latched) and zero-extend.
  reg  [31:0] lsu_rdata_r;
  wire [1:0]  byte_off = alu_result[1:0];

  // Build store word / wmask combinationally.
  reg  [31:0] store_word;
  reg  [3:0]  store_wmask;
  reg  [1:0]  store_size;
  always @(*) begin
    store_word  = 32'h0;
    store_wmask = 4'b0;
    store_size  = 2'b10; // default sw
    if (mem_we) begin
      case (funct3)
        3'b010: begin                                       // sw
          store_word  = rs2_val;
          store_wmask = 4'b1111;
          store_size  = 2'b10;
        end
        3'b000: begin                                       // sb
          case (byte_off)
            2'b00: begin store_word = {24'h0, rs2_val[7:0]};                 store_wmask = 4'b0001; end
            2'b01: begin store_word = {16'h0, rs2_val[7:0], 8'h0};            store_wmask = 4'b0010; end
            2'b10: begin store_word = {8'h0,  rs2_val[7:0], 16'h0};           store_wmask = 4'b0100; end
            2'b11: begin store_word = {       rs2_val[7:0], 24'h0};           store_wmask = 4'b1000; end
          endcase
          store_size = 2'b00;
        end
        default: begin
          store_word  = rs2_val;
          store_wmask = 4'b1111;
          store_size  = 2'b10;
        end
      endcase
    end
  end

  // Build read size for loads.
  reg [1:0] load_size;
  always @(*) begin
    case (funct3)
      3'b010: load_size = 2'b10; // lw
      3'b100: load_size = 2'b00; // lbu
      default: load_size = 2'b10;
    endcase
  end

  // Decode load result (use latched rdata).
  reg [31:0] load_data;
  always @(*) begin
    load_data = 32'h0;
    if (mem_re) begin
      case (funct3)
        3'b010: load_data = lsu_rdata_r;
        3'b100: begin
          case (byte_off)
            2'b00: load_data = {24'h0, lsu_rdata_r[ 7: 0]};
            2'b01: load_data = {24'h0, lsu_rdata_r[15: 8]};
            2'b10: load_data = {24'h0, lsu_rdata_r[23:16]};
            2'b11: load_data = {24'h0, lsu_rdata_r[31:24]};
          endcase
        end
        default: load_data = lsu_rdata_r;
      endcase
    end
  end

  // ---- CSR ------------------------------------------------------------------
  wire [31:0] csr_rdata;
  wire [31:0] csr_src = csr_use_uimm ? {27'b0, csr_uimm} : rs1_val;
  wire [31:0] csr_wdata =
        (csr_op == 2'b00) ? csr_src :              // csrrw / csrrwi
        (csr_op == 2'b01) ? (csr_rdata | csr_src): // csrrs / csrrsi
        (csr_op == 2'b10) ? (csr_rdata & ~csr_src):// csrrc / csrrci
                            csr_src;
  wire csr_we_wb = csr_we_dec & (state == S_WB) & ~is_ebreak;
  CSR u_csr (
    .clk(clock), .rst(reset),
    .csr_raddr(csr_addr), .csr_rdata(csr_rdata),
    .csr_we(csr_we_wb),
    .csr_waddr(csr_addr), .csr_wdata(csr_wdata)
  );

  // ---- WBU + PC -------------------------------------------------------------
  wire [31:0] pc_next;
  WBU u_wbu (
    .alu_result(alu_result), .load_data(load_data),
    .csr_rdata(csr_rdata),
    .wb_sel(wb_sel), .is_jalr(is_jalr),
    .pc(pc),
    .pc_next(pc_next), .wb_data(wb_data)
  );

  // ---- Bus outputs ----------------------------------------------------------
  // Hold req high while waiting for resp. The bridge guarantees a single
  // handshake per request.
  assign io_ifu_reqValid = (state == S_IF) & ~reset;
  assign io_ifu_addr     = {pc[31:2], 2'b00};

  wire ls_active = (state == S_LS);
  assign io_lsu_reqValid = ls_active & ~reset & (mem_re | mem_we);
  // io_lsu_addr is the byte address per cpu-interface.md. Devices (APB UART
  // in particular) use the low 3 bits to index their register file, so we
  // MUST NOT zero them. The wmask + size already encode which bytes inside
  // the word are active.
  assign io_lsu_addr     = alu_result;
  assign io_lsu_size     = mem_we ? store_size : load_size;
  assign io_lsu_wen      = mem_we;
  assign io_lsu_wdata    = store_word;
  assign io_lsu_wmask    = store_wmask;

  // ---- FSM next-state -------------------------------------------------------
  always @(*) begin
    next_state = state;
    case (state)
      S_IF: if (io_ifu_respValid) next_state = S_EX;
      S_EX: begin
        if (is_ebreak)                 next_state = S_EX; // park forever on ebreak
        else if (mem_re | mem_we)      next_state = S_LS;
        else                           next_state = S_WB;
      end
      S_LS: if (io_lsu_respValid)      next_state = S_WB;
      S_WB:                            next_state = S_IF;
      default:                         next_state = S_IF;
    endcase
  end

  // ---- Sequential updates ---------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      state       <= S_IF;
      pc          <= `SOC_PC_RESET_VEC;
      inst_r      <= 32'h0;
      lsu_rdata_r <= 32'h0;
    end else begin
      state <= next_state;
      // Latch instruction when fetch handshake completes.
      if (state == S_IF && io_ifu_respValid) begin
        inst_r <= io_ifu_rdata;
      end
      // Latch load data when LSU handshake completes (read path).
      if (state == S_LS && io_lsu_respValid && mem_re) begin
        lsu_rdata_r <= io_lsu_rdata;
      end
      // Update PC on writeback only.
      if (state == S_WB && !is_ebreak) begin
        pc <= pc_next;
      end
    end
  end
endmodule
