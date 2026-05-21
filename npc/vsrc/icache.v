// =============================================================================
// icache  --  B4a 简易直接映射 instruction cache
// =============================================================================
// 块大小 = 4B (一条指令), 块数 = 2**BLOCKS_LOG (默认 16), 直接映射, 只读.
// 存储阵列用触发器 (不依赖 SRAM IP).
//
// 上游 (CPU 侧): 半握手 SimpleBus (与 ysyx_22040000.v 的 io_ifu_* 一致)
//   req_valid     -->   (CPU 持续举手, 直到看到 resp_valid)
//   req_addr      -->
//   resp_valid    <--   (cache 一拍脉冲: hit 时与 req_valid 同周期; miss 时
//                        等下游 bus 回数后那一拍)
//   resp_data     <--
//
// 下游 (Bus 侧): 同样的半握手, 接到 ysyxSoC MemBridge
//   bus_req_valid  -->
//   bus_req_addr   -->
//   bus_resp_valid <--
//   bus_resp_data  <--
//
// 地址切分 (32 位):
//   [31 : OFFSET_LOG+BLOCKS_LOG] = tag
//   [OFFSET_LOG+BLOCKS_LOG-1 : OFFSET_LOG] = index
//   [OFFSET_LOG-1 : 0] = offset
//
// 例如 BLOCKS_LOG=4, OFFSET_LOG=2 时:
//   [31:6] = tag (26b)
//   [5:2]  = index (4b, 16 行)
//   [1:0]  = offset (2b, 始终对齐, 不用)
//
// FSM:
//   S_IDLE : 无外发请求. 上游若 req_valid 且 hit, 同周期组合返回 resp_valid.
//            若 req_valid 且 miss, 下一拍进 S_MISS.
//   S_MISS : 拉 bus_req_valid=1. SimpleBus 的 MemBridge 同样要求 reqValid 一
//            直拉到 respValid 那拍. 收到 bus_resp_valid 后, 同周期填表
//            (写入 data + tag + valid), 同周期向上游回应 resp_valid, 下一
//            拍回 S_IDLE.
// =============================================================================
module icache #(
  parameter BLOCKS_LOG = 4,        // 块数 = 2^BLOCKS_LOG, 默认 16
  parameter OFFSET_LOG = 2         // 块大小 = 2^OFFSET_LOG 字节, 默认 4
)(
  input              clock,
  input              reset,
  // ---- 上游: CPU 取指 ------------------------------------------------------
  input              req_valid,
  input       [31:0] req_addr,
  output             resp_valid,
  output      [31:0] resp_data,
  // ---- 下游: SimpleBus master ---------------------------------------------
  output             bus_req_valid,
  output      [31:0] bus_req_addr,
  input              bus_resp_valid,
  input       [31:0] bus_resp_data
);
  // ---- 派生参数 ------------------------------------------------------------
  localparam BLOCKS    = (1 << BLOCKS_LOG);
  localparam TAG_LSB   = OFFSET_LOG + BLOCKS_LOG;   // tag 的最低有效位
  localparam TAG_WIDTH = 32 - TAG_LSB;

  // ---- 切分上游地址 --------------------------------------------------------
  wire [TAG_WIDTH-1:0]  req_tag   = req_addr[31:TAG_LSB];
  wire [BLOCKS_LOG-1:0] req_index = req_addr[TAG_LSB-1:OFFSET_LOG];

  // ---- 存储阵列 (触发器) ---------------------------------------------------
  reg [31:0]           data_array [0:BLOCKS-1];
  reg [TAG_WIDTH-1:0]  tag_array  [0:BLOCKS-1];
  reg                  valid_array[0:BLOCKS-1];

  // ---- 组合查 hit ----------------------------------------------------------
  wire entry_valid = valid_array[req_index];
  wire tag_match   = (tag_array[req_index] == req_tag);
  wire hit         = req_valid & entry_valid & tag_match;

  // ---- FSM -----------------------------------------------------------------
  localparam S_IDLE = 1'b0;
  localparam S_MISS = 1'b1;
  reg state, next_state;

  always @(*) begin
    next_state = state;
    case (state)
      S_IDLE: if (req_valid & ~hit)  next_state = S_MISS;
      S_MISS: if (bus_resp_valid)    next_state = S_IDLE;
      default:                       next_state = S_IDLE;
    endcase
  end

  // ---- 外部端口 ------------------------------------------------------------
  // 上游: hit 时同周期回应; miss 时等 S_MISS 拿到 bus_resp_valid 那一拍.
  wire idle_hit  = (state == S_IDLE) & hit;
  wire miss_done = (state == S_MISS) & bus_resp_valid;
  assign resp_valid = idle_hit | miss_done;
  assign resp_data  = miss_done ? bus_resp_data : data_array[req_index];

  // 下游: miss 时把上游地址原样转发给 bus.
  assign bus_req_valid = (state == S_MISS);
  assign bus_req_addr  = {req_addr[31:OFFSET_LOG], {OFFSET_LOG{1'b0}}};

  // ---- 性能计数器 ----------------------------------------------------------
  // 每次完成一次取指 (即 resp_valid=1 那一拍) 都计一次 access; 当时是 idle_hit
  // 就计 hit, 否则 (miss_done) 计 miss. 不需要边沿检测.
  // /*verilator public_flat_rd*/ 让 verilator 在保持顶层 root 扁平符号结构
  // (即 *__DOT__* 风格) 的同时把这三个寄存器导出, 这样不会强制 verilator
  // 给整个层次按模块拆类, 主仿真 C++ 不用修改取 inst_r/state/u_rf 的路径.
  reg [63:0] cnt_access /*verilator public_flat_rd*/;
  reg [63:0] cnt_hit    /*verilator public_flat_rd*/;
  reg [63:0] cnt_miss   /*verilator public_flat_rd*/;

  always @(posedge clock) begin
    if (reset) begin
      cnt_access <= 64'h0;
      cnt_hit    <= 64'h0;
      cnt_miss   <= 64'h0;
    end else if (resp_valid) begin
      cnt_access <= cnt_access + 64'h1;
      if (idle_hit) cnt_hit  <= cnt_hit  + 64'h1;
      else          cnt_miss <= cnt_miss + 64'h1;
    end
  end

  // ---- 主时序 --------------------------------------------------------------
  integer i;
  always @(posedge clock) begin
    if (reset) begin
      state <= S_IDLE;
      for (i = 0; i < BLOCKS; i = i + 1) begin
        valid_array[i] <= 1'b0;
        tag_array[i]   <= {TAG_WIDTH{1'b0}};
        data_array[i]  <= 32'h0;
      end
    end else begin
      state <= next_state;
      // miss 填表: 把 bus_resp_data 写到 [req_index].
      if (miss_done) begin
        valid_array[req_index] <= 1'b1;
        tag_array[req_index]   <= req_tag;
        data_array[req_index]  <= bus_resp_data;
      end
    end
  end

  // ---- verilator 防止误报未用 ----------------------------------------------
  // (cnt_access 等仅供 C 侧 DPI 读取, RTL 内部无 sink)
  // verilator lint_off UNUSED
  wire _unused_cnt = &{1'b0, cnt_access, cnt_hit, cnt_miss};
  // verilator lint_on UNUSED
endmodule
