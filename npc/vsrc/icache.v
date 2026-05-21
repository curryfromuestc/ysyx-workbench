// =============================================================================
// icache  --  B4b w-way set-associative + true-LRU instruction cache
// =============================================================================
// 参数化:
//   BLOCKS_LOG = log2(总块数),     默认 4 -> 16 块
//   WAYS_LOG   = log2(每 set 路数), 默认 1 -> 2-way (B4a 用 0 -> 1-way 退化即直接映射)
//   OFFSET_LOG = log2(块大小字节), 默认 2 -> 4B (一条指令一个块)
// 派生:
//   WAYS      = 2 ^^ WAYS_LOG
//   SETS_LOG  = BLOCKS_LOG - WAYS_LOG
//   SETS      = 2 ^^ SETS_LOG
//   TAG_LSB   = OFFSET_LOG + SETS_LOG
//   TAG_W     = 32 - TAG_LSB
//   AGE_W     = max(WAYS_LOG, 1)  -- age 计数器位宽, WAYS=1 时仍保留 1 位避免 [-1:0]
//   IDX_W     = WAYS_LOG + 1      -- way 索引位宽, WAYS=1 时仍保留 1 位避免 [-1:0]
//
// 地址切分:
//   [31 : TAG_LSB]               = tag
//   [TAG_LSB-1 : OFFSET_LOG]     = index (set 号)
//   [OFFSET_LOG-1 : 0]           = offset (块内偏移, 块大小 = 4B 时弃用)
//
// 存储 (触发器, 不依赖 SRAM IP):
//   data_arr  [SETS][WAYS] -- 32 b
//   tag_arr   [SETS][WAYS] -- TAG_W b
//   valid_arr [SETS][WAYS] -- 1 b
//   age_arr   [SETS][WAYS] -- AGE_W b  (per-way LRU age 计数, 0 = MRU, WAYS-1 = LRU)
//
// 真 LRU 更新规则 (每个 set 独立):
//   访问 / 填充 way h 时:
//     for w in 0..WAYS-1:
//       if (w == h)                age[w] <= 0;        // 命中/填充的路设为 MRU
//       else if (age[w] < age[h])  age[w] <= age[w]+1; // 比 h 更新的全部老化一格
//       else                       age[w] unchanged;   // 比 h 更老的不动
//   不变量: 每个 set 内 age 始终是 0..WAYS-1 的一个排列.
//   替换: victim = way s.t. age == WAYS-1.
//
// FSM (与 B4a 同结构, 2 态):
//   S_IDLE: 上游 req_valid:
//           - hit  -> 同周期组合返回 resp_valid + resp_data; 更新该 set 的 LRU.
//           - miss -> 选 victim way, 进 S_MISS.
//   S_MISS: 拉 bus_req_valid=1 -> 等 bus_resp_valid:
//           - 同周期 resp_valid=1, 透传 bus_resp_data;
//           - 同周期填表: data/tag/valid 写到 victim way; 更新 LRU; 回 S_IDLE.
//
// 性能计数器 (verilator public_flat_rd):
//   cnt_access / cnt_hit / cnt_miss : 同 B4a 含义.
//   cnt_victim : 选 victim 次数 (理论上 == miss 数, sanity check).
//
// 兼容 WAYS=1 (= 直接映射, B4a 退化场景):
//   age_arr 是 1 位但永远为 0; victim_way 永远 = 0; LRU 更新逻辑也始终把 way0
//   设为 MRU, 等价于不动.
// =============================================================================
module icache #(
  parameter BLOCKS_LOG = 4,
  parameter WAYS_LOG   = 1,
  parameter OFFSET_LOG = 2
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
  localparam WAYS      = (1 << WAYS_LOG);
  localparam SETS_LOG  = BLOCKS_LOG - WAYS_LOG;
  localparam SETS      = (1 << SETS_LOG);
  localparam TAG_LSB   = OFFSET_LOG + SETS_LOG;
  localparam TAG_W     = 32 - TAG_LSB;
  // 为兼容 WAYS=1 (WAYS_LOG=0), 不可声明 [-1:0]; 用 max(.,1) 兜底.
  localparam AGE_W     = (WAYS_LOG == 0) ? 1 : WAYS_LOG;
  localparam IDX_W     = (WAYS_LOG == 0) ? 1 : WAYS_LOG;

  // ---- 切分上游地址 --------------------------------------------------------
  wire [TAG_W-1:0]    req_tag   = req_addr[31:TAG_LSB];
  wire [SETS_LOG-1:0] req_index = req_addr[TAG_LSB-1:OFFSET_LOG];

  // ---- 存储阵列 (触发器) ---------------------------------------------------
  reg [31:0]      data_arr [0:SETS-1][0:WAYS-1];
  reg [TAG_W-1:0] tag_arr  [0:SETS-1][0:WAYS-1];
  reg             valid_arr[0:SETS-1][0:WAYS-1];
  reg [AGE_W-1:0] age_arr  [0:SETS-1][0:WAYS-1];

  // ---- 组合: 并行比较所有 way 的 (valid && tag==req_tag) ------------------
  wire [WAYS-1:0] way_hit_vec;
  genvar gw;
  generate
    for (gw = 0; gw < WAYS; gw = gw + 1) begin : g_hit_check
      assign way_hit_vec[gw] = valid_arr[req_index][gw]
                            & (tag_arr[req_index][gw] == req_tag);
    end
  endgenerate
  wire hit = req_valid & (|way_hit_vec);

  // hit way 编码: one-hot -> binary. 正确实现下同 set 同 tag 只命中一路.
  reg [IDX_W-1:0] hit_way;
  integer ih;
  always @(*) begin
    hit_way = {IDX_W{1'b0}};
    for (ih = 0; ih < WAYS; ih = ih + 1) begin
      if (way_hit_vec[ih]) hit_way = ih[IDX_W-1:0];
    end
  end

  // victim 选择 (规范性): 先扫一遍 invalid way; 若有 invalid 就挑它 (避免无谓
  // 踢一个合法表项). 没有 invalid 时再退化到 LRU = (age == WAYS-1) 的那一路.
  // WAYS=1 时 WAYS-1=0, way0 始终 age=0, 永远是 victim. 已通过 W=1 退化等效
  // B4a 验证 -- 加 invalid-first 后, 因 age 初值排列 (wi=>age=wi) 已经让 cold
  // start 期间 victim 与 invalid-first 路径一致, 所以实测值不变, 仅是语义增强.
  reg [IDX_W-1:0] victim_way;
  reg             has_invalid;
  integer iv;
  always @(*) begin
    victim_way  = {IDX_W{1'b0}};
    has_invalid = 1'b0;
    // 第一遍: 找最低编号的 invalid way (优先编码).
    for (iv = 0; iv < WAYS; iv = iv + 1) begin
      if (!valid_arr[req_index][iv] & ~has_invalid) begin
        victim_way  = iv[IDX_W-1:0];
        has_invalid = 1'b1;
      end
    end
    // 第二遍: 全 valid 时退化到 LRU = age==WAYS-1 那一路.
    if (!has_invalid) begin
      for (iv = 0; iv < WAYS; iv = iv + 1) begin
        if (age_arr[req_index][iv] == (WAYS-1)) victim_way = iv[IDX_W-1:0];
      end
    end
  end

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
  wire idle_hit  = (state == S_IDLE) & hit;
  wire miss_done = (state == S_MISS) & bus_resp_valid;
  assign resp_valid = idle_hit | miss_done;
  assign resp_data  = miss_done ? bus_resp_data
                                : data_arr[req_index][hit_way];

  assign bus_req_valid = (state == S_MISS);
  assign bus_req_addr  = {req_addr[31:OFFSET_LOG], {OFFSET_LOG{1'b0}}};

  // ---- 性能计数器 ----------------------------------------------------------
  reg [63:0] cnt_access /*verilator public_flat_rd*/;
  reg [63:0] cnt_hit    /*verilator public_flat_rd*/;
  reg [63:0] cnt_miss   /*verilator public_flat_rd*/;
  reg [63:0] cnt_victim /*verilator public_flat_rd*/;

  always @(posedge clock) begin
    if (reset) begin
      cnt_access <= 64'h0;
      cnt_hit    <= 64'h0;
      cnt_miss   <= 64'h0;
      cnt_victim <= 64'h0;
    end else begin
      if (resp_valid) begin
        cnt_access <= cnt_access + 64'h1;
        if (idle_hit) cnt_hit  <= cnt_hit  + 64'h1;
        else          cnt_miss <= cnt_miss + 64'h1;
      end
      if (miss_done) cnt_victim <= cnt_victim + 64'h1;
    end
  end

  // ---- 主时序: 状态 + 填表 + LRU 更新 -------------------------------------
  // 触发 LRU 更新的事件:
  //   idle_hit  -> touched_way = hit_way
  //   miss_done -> touched_way = victim_way (新填进的路, 视为 MRU)
  wire [IDX_W-1:0] touched_way = idle_hit ? hit_way : victim_way;
  // 把 touched_way 此刻的 age 提到组合层 (因为 always 内 age_arr[touched_way]
  // 是个变量索引, Verilator 会展开 mux; 提到 wire 一来可读, 二来 quartus/yosys
  // 也能识别).
  wire [AGE_W-1:0] touched_age = age_arr[req_index][touched_way];

  integer si, wi;
  always @(posedge clock) begin
    if (reset) begin
      state <= S_IDLE;
      for (si = 0; si < SETS; si = si + 1) begin
        for (wi = 0; wi < WAYS; wi = wi + 1) begin
          valid_arr[si][wi] <= 1'b0;
          tag_arr  [si][wi] <= {TAG_W{1'b0}};
          data_arr [si][wi] <= 32'h0;
          // 初始 age: way wi 的 age = wi. 这样 way 0 最 MRU, way WAYS-1 最 LRU,
          // 第一次 miss 一定挑 way WAYS-1 当 victim. WAYS=1 时 wi=0, age=0.
          age_arr  [si][wi] <= wi[AGE_W-1:0];
        end
      end
    end else begin
      state <= next_state;

      // miss 填表: 写到 victim way.
      if (miss_done) begin
        valid_arr[req_index][victim_way] <= 1'b1;
        tag_arr  [req_index][victim_way] <= req_tag;
        data_arr [req_index][victim_way] <= bus_resp_data;
      end

      // LRU 更新: 不管命中还是 miss 填回, 都把 touched_way 设为 MRU (age=0),
      // 比 touched_way 旧的 (age < touched_age) 不动, 比 touched_way 新的 (age <
      // touched_age 但因 touched 即将变 0 我们让它们 age+1, 即 "比 touched 更新
      // 或与 touched 同新" 的都 +1) -> 严格说: 原 age < touched_age 的 +1.
      // 不变量证明: 原排列 {0,1,..,WAYS-1}, touched 的 age = k. 处理后:
      //   原 age < k 的 way 都 +1 -> 占据 1..k
      //   touched way -> 0
      //   原 age > k 的 way 不变 -> 占据 k+1..WAYS-1
      //   合起来还是 0..WAYS-1 的排列.
      if (idle_hit | miss_done) begin
        for (wi = 0; wi < WAYS; wi = wi + 1) begin
          if (wi[IDX_W-1:0] == touched_way) begin
            age_arr[req_index][wi] <= {AGE_W{1'b0}};
          end else if (age_arr[req_index][wi] < touched_age) begin
            age_arr[req_index][wi] <= age_arr[req_index][wi] + 1'b1;
          end
        end
      end
    end
  end

endmodule
