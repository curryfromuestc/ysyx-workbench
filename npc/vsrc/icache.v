// =============================================================================
// icache  --  B4c w-way set-associative + true-LRU + multi-beat fill
// =============================================================================
// 参数化:
//   BLOCKS_LOG = log2(总块数),     默认 5 -> 32 块
//   WAYS_LOG   = log2(每 set 路数), 默认 1 -> 2-way
//   OFFSET_LOG = log2(块大小字节), 默认 5 -> 32 B / 块 (8 个 32-bit word)
// 派生:
//   WAYS         = 2 ^^ WAYS_LOG
//   SETS_LOG     = BLOCKS_LOG - WAYS_LOG
//   SETS         = 2 ^^ SETS_LOG
//   BLOCK_BYTES  = 2 ^^ OFFSET_LOG
//   BLOCK_WORDS  = BLOCK_BYTES / 4
//   WORD_LOG     = OFFSET_LOG - 2     -- log2(BLOCK_WORDS), 必须 >= 0
//   TAG_LSB      = OFFSET_LOG + SETS_LOG
//   TAG_W        = 32 - TAG_LSB
//   AGE_W        = max(WAYS_LOG, 1)   -- 详见 B4b
//   IDX_W        = max(WAYS_LOG, 1)
//   WCNT_W       = max(WORD_LOG, 1)   -- fill_cnt 位宽兜底
//
// 地址切分 (B4c 引入字偏移):
//   [31 : TAG_LSB]                       = tag
//   [TAG_LSB-1 : OFFSET_LOG]             = index (set 号)
//   [OFFSET_LOG-1 : 2]                   = word_off (块内第几个 word)
//   [1:0]                                = byte 偏移 (我们 PC 只对齐到 4B, 永远 = 0)
//
// 存储 (触发器, 不依赖 SRAM IP):
//   data_arr  [SETS][WAYS] -- BLOCK_BITS = BLOCK_WORDS * 32 b
//   tag_arr   [SETS][WAYS] -- TAG_W b
//   valid_arr [SETS][WAYS] -- 1 b
//   age_arr   [SETS][WAYS] -- AGE_W b  (per-way LRU age, 0 = MRU, WAYS-1 = LRU)
//
// 真 LRU + invalid-first 替换: 与 B4b 一致 (详见 B4b 报告).
//
// FSM (B4c 升级为 3 态, multi-beat fill):
//   S_IDLE:  上游 req_valid 来:
//            - hit  -> 同周期组合返回 resp_valid + resp_data (从命中块按
//              word_off 选出 32-bit). LRU 更新.
//            - miss -> 进 S_FILL, fill_cnt = 0, fill 基地址锁存到 fill_base.
//   S_FILL:  连续对 bus 发起 BLOCK_WORDS 次 32-bit 读, 每收到一拍 resp 就
//            把数据写到该 way 的 data_arr 的 fill_cnt 这个 word slot. 当
//            fill_cnt == BLOCK_WORDS-1 时下一拍进 S_DONE, 同时把 valid/tag
//            置好, LRU 更新.
//   S_DONE:  组合输出 resp_valid + resp_data (从填好的块按 word_off 选出
//            32-bit). 单拍后回 S_IDLE. 之所以需要单独一拍, 是因为 S_FILL
//            末拍写表是非阻塞 (<= ), 同周期组合从 data_arr 读出来不会拿到
//            刚写入的最后一个 word; 把回 resp 推迟一拍.
//
// 性能计数器 (verilator public_flat_rd):
//   cnt_access / cnt_hit / cnt_miss / cnt_victim : 同 B4b.
//   每个 miss 计 1 次 (不是每 beat 计 1 次), 即 access = hit + miss.
//
// 兼容退化:
//   - OFFSET_LOG=2 (4B 块): BLOCK_WORDS=1, WORD_LOG=0, 等价于 B4b. FSM 走到
//     S_FILL 后立即收到 resp, fill_cnt 直接到 BLOCK_WORDS-1, 进 S_DONE; 与
//     B4b 的 S_MISS 行为一致 (多花 1 拍 S_DONE).
//   - WAYS_LOG=0 (直接映射): age_arr 退化, 行为同 B4a 直映 (除了块大小).
// =============================================================================
module icache #(
  parameter BLOCKS_LOG = 5,
  parameter WAYS_LOG   = 1,
  parameter OFFSET_LOG = 5
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
  localparam WAYS         = (1 << WAYS_LOG);
  localparam SETS_LOG     = BLOCKS_LOG - WAYS_LOG;
  localparam SETS         = (1 << SETS_LOG);
  localparam BLOCK_BYTES  = (1 << OFFSET_LOG);
  localparam BLOCK_WORDS  = (BLOCK_BYTES / 4);
  localparam BLOCK_BITS   = BLOCK_WORDS * 32;
  localparam WORD_LOG     = OFFSET_LOG - 2;  // == log2(BLOCK_WORDS), 要求 >=0
  localparam TAG_LSB      = OFFSET_LOG + SETS_LOG;
  localparam TAG_W        = 32 - TAG_LSB;
  // 为兼容 WAYS=1 (WAYS_LOG=0) / BLOCK_WORDS=1 (WORD_LOG=0), 不可声明 [-1:0].
  localparam AGE_W        = (WAYS_LOG == 0) ? 1 : WAYS_LOG;
  localparam IDX_W        = (WAYS_LOG == 0) ? 1 : WAYS_LOG;
  localparam WCNT_W       = (WORD_LOG == 0) ? 1 : WORD_LOG;

  // ---- 切分上游地址 --------------------------------------------------------
  wire [TAG_W-1:0]    req_tag    = req_addr[31:TAG_LSB];
  wire [SETS_LOG-1:0] req_index  = req_addr[TAG_LSB-1:OFFSET_LOG];
  // word_off 取 [OFFSET_LOG-1:2]. WORD_LOG=0 时这段是空的, 用 1'b0 占位.
  wire [WCNT_W-1:0]   req_woff;
  generate
    if (WORD_LOG == 0) begin : g_woff_zero
      assign req_woff = 1'b0;
    end else begin : g_woff_real
      assign req_woff = req_addr[OFFSET_LOG-1:2];
    end
  endgenerate

  // ---- 存储阵列 (触发器) ---------------------------------------------------
  reg [BLOCK_BITS-1:0] data_arr [0:SETS-1][0:WAYS-1];
  reg [TAG_W-1:0]      tag_arr  [0:SETS-1][0:WAYS-1];
  reg                  valid_arr[0:SETS-1][0:WAYS-1];
  reg [AGE_W-1:0]      age_arr  [0:SETS-1][0:WAYS-1];

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

  // hit way 编码: one-hot -> binary.
  reg [IDX_W-1:0] hit_way;
  integer ih;
  always @(*) begin
    hit_way = {IDX_W{1'b0}};
    for (ih = 0; ih < WAYS; ih = ih + 1) begin
      if (way_hit_vec[ih]) hit_way = ih[IDX_W-1:0];
    end
  end

  // victim 选择: invalid-first, 否则 LRU.
  reg [IDX_W-1:0] victim_way;
  reg             has_invalid;
  integer iv;
  always @(*) begin
    victim_way  = {IDX_W{1'b0}};
    has_invalid = 1'b0;
    for (iv = 0; iv < WAYS; iv = iv + 1) begin
      if (!valid_arr[req_index][iv] & ~has_invalid) begin
        victim_way  = iv[IDX_W-1:0];
        has_invalid = 1'b1;
      end
    end
    if (!has_invalid) begin
      for (iv = 0; iv < WAYS; iv = iv + 1) begin
        if (age_arr[req_index][iv] == (WAYS-1)) victim_way = iv[IDX_W-1:0];
      end
    end
  end

  // ---- FSM -----------------------------------------------------------------
  localparam S_IDLE = 2'd0;
  localparam S_FILL = 2'd1;
  localparam S_DONE = 2'd2;
  reg [1:0] state, next_state;

  // fill_cnt: 本次 miss 已收到的 word 数. 范围 [0, BLOCK_WORDS-1].
  reg [WCNT_W-1:0] fill_cnt;
  // miss 入口时锁存 set / tag / victim / fill 基地址 (= 块基地址) /
  // miss 时的 word_off (用于 S_DONE 阶段返回数据 -- req_addr 在 fill 期间
  // 上游 FSM 是稳定的, 但锁存一遍更稳妥).
  reg [SETS_LOG-1:0] miss_index_r;
  reg [TAG_W-1:0]    miss_tag_r;
  reg [IDX_W-1:0]    miss_way_r;
  reg [WCNT_W-1:0]   miss_woff_r;
  reg [31:0]         miss_base_r;

  // fill_last = 当前接收到的是最后一拍.
  wire fill_last = bus_resp_valid & (fill_cnt == (BLOCK_WORDS-1));

  always @(*) begin
    next_state = state;
    case (state)
      S_IDLE: if (req_valid & ~hit)         next_state = S_FILL;
      S_FILL: if (fill_last)                next_state = S_DONE;
      S_DONE:                               next_state = S_IDLE;
      default:                              next_state = S_IDLE;
    endcase
  end

  // ---- 外部端口 ------------------------------------------------------------
  // idle_hit: S_IDLE + hit, 同周期组合返回.
  // done_resp: S_DONE 那一拍, 块已填好, 从 data_arr 按 miss_woff_r 取 word.
  wire idle_hit  = (state == S_IDLE) & hit;
  wire done_resp = (state == S_DONE);

  // 从命中块里按 word_off 选 word. 用一个跨 way / 跨 woff 的 mux.
  // 命中分支用 hit_way + req_woff; S_DONE 分支用 miss_way_r + miss_woff_r.
  wire [BLOCK_BITS-1:0] hit_block  = data_arr[req_index][hit_way];
  wire [BLOCK_BITS-1:0] done_block = data_arr[miss_index_r][miss_way_r];

  // 取出第 woff 个 word: block[32*(woff+1)-1 : 32*woff].
  // Verilog 不允许动态 +: 起点直接来自 reg, 需要 *32. WORD_LOG=0 时直接取低 32.
  reg [31:0] hit_word;
  reg [31:0] done_word;
  integer iw;
  always @(*) begin
    hit_word  = 32'h0;
    done_word = 32'h0;
    if (BLOCK_WORDS == 1) begin
      hit_word  = hit_block[31:0];
      done_word = done_block[31:0];
    end else begin
      for (iw = 0; iw < BLOCK_WORDS; iw = iw + 1) begin
        if (iw[WCNT_W-1:0] == req_woff)    hit_word  = hit_block [iw*32 +: 32];
        if (iw[WCNT_W-1:0] == miss_woff_r) done_word = done_block[iw*32 +: 32];
      end
    end
  end

  assign resp_valid = idle_hit | done_resp;
  assign resp_data  = done_resp ? done_word : hit_word;

  // S_FILL 期间持续拉 bus_req_valid, bus_req_addr = 当前 word 的字节地址.
  // 我们一拍一拍发, 每收到 resp_valid 就把 fill_cnt+1, 同时把 req_addr 推进 4B.
  // miss_base_r 在进入 S_FILL 的同一拍 latch 成块基地址.
  assign bus_req_valid = (state == S_FILL);
  assign bus_req_addr  = miss_base_r + ({{(32-WCNT_W-2){1'b0}}, fill_cnt, 2'b00});

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
      // access + hit/miss 在 resp 那一拍累加 (每次取指最多算一次).
      if (resp_valid) begin
        cnt_access <= cnt_access + 64'h1;
        if (idle_hit) cnt_hit  <= cnt_hit  + 64'h1;
        else          cnt_miss <= cnt_miss + 64'h1;
      end
      // 选 victim 计数: 在 fill 完成那一拍累加 (== S_FILL 最后一 beat).
      // 与 cnt_miss 同步, 保持 sanity check 不变量 victim == miss.
      if ((state == S_FILL) & fill_last) cnt_victim <= cnt_victim + 64'h1;
    end
  end

  // ---- 主时序: 状态 + 填表 + LRU 更新 -------------------------------------
  // touched_way: idle_hit 时 = hit_way; S_DONE 那一拍 = miss_way_r.
  // 在 idle_hit 时立即更新 LRU; 在 S_DONE 那一拍把刚 fill 的 way 设为 MRU.
  wire is_lru_touch = idle_hit | done_resp;
  wire [SETS_LOG-1:0] touched_index = idle_hit ? req_index : miss_index_r;
  wire [IDX_W-1:0]    touched_way   = idle_hit ? hit_way   : miss_way_r;
  wire [AGE_W-1:0]    touched_age   = age_arr[touched_index][touched_way];

  integer si, wi;
  always @(posedge clock) begin
    if (reset) begin
      state        <= S_IDLE;
      fill_cnt     <= {WCNT_W{1'b0}};
      miss_index_r <= {SETS_LOG{1'b0}};
      miss_tag_r   <= {TAG_W{1'b0}};
      miss_way_r   <= {IDX_W{1'b0}};
      miss_woff_r  <= {WCNT_W{1'b0}};
      miss_base_r  <= 32'h0;
      for (si = 0; si < SETS; si = si + 1) begin
        for (wi = 0; wi < WAYS; wi = wi + 1) begin
          valid_arr[si][wi] <= 1'b0;
          tag_arr  [si][wi] <= {TAG_W{1'b0}};
          data_arr [si][wi] <= {BLOCK_BITS{1'b0}};
          // 初始 age: way wi 的 age = wi (第一次 miss 优先选 way WAYS-1).
          age_arr  [si][wi] <= wi[AGE_W-1:0];
        end
      end
    end else begin
      state <= next_state;

      // miss 入口: 在 S_IDLE 看到 miss 时锁存 set/tag/victim/word_off/base.
      if ((state == S_IDLE) & req_valid & ~hit) begin
        miss_index_r <= req_index;
        miss_tag_r   <= req_tag;
        miss_way_r   <= victim_way;
        miss_woff_r  <= req_woff;
        // 块基地址: req_addr 的低 OFFSET_LOG 位清零.
        miss_base_r  <= {req_addr[31:OFFSET_LOG], {OFFSET_LOG{1'b0}}};
        fill_cnt     <= {WCNT_W{1'b0}};
      end

      // S_FILL 收到一拍 resp: 写入 data_arr 的对应 word slot, fill_cnt+1.
      // 最后一拍同时把 valid / tag 写好 (LRU 在 done_resp 那一拍更新).
      if ((state == S_FILL) & bus_resp_valid) begin
        if (BLOCK_WORDS == 1) begin
          data_arr[miss_index_r][miss_way_r][31:0] <= bus_resp_data;
        end else begin
          // 动态写入 word slot: 因为 +: 起点不能是变量, 走 case-on-fill_cnt.
          // 用一段 for 循环展开等价 case.
          for (iw = 0; iw < BLOCK_WORDS; iw = iw + 1) begin
            if (iw[WCNT_W-1:0] == fill_cnt)
              data_arr[miss_index_r][miss_way_r][iw*32 +: 32] <= bus_resp_data;
          end
        end
        if (fill_last) begin
          valid_arr[miss_index_r][miss_way_r] <= 1'b1;
          tag_arr  [miss_index_r][miss_way_r] <= miss_tag_r;
        end
        fill_cnt <= fill_cnt + {{(WCNT_W-1){1'b0}}, 1'b1};
      end

      // LRU 更新 (规则与 B4b 一致).
      if (is_lru_touch) begin
        for (wi = 0; wi < WAYS; wi = wi + 1) begin
          if (wi[IDX_W-1:0] == touched_way) begin
            age_arr[touched_index][wi] <= {AGE_W{1'b0}};
          end else if (age_arr[touched_index][wi] < touched_age) begin
            age_arr[touched_index][wi] <= age_arr[touched_index][wi] + 1'b1;
          end
        end
      end
    end
  end

endmodule
