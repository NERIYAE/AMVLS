

//------------------------------------------------------------------------------
// car_node.sv
// Single car physics + safety node.
// - Uses 1 kHz tick from tick_if (clk_100m, rst_n, tick_en, tick_id).
// - All state (SAFE_HALT, heartbeat, x/v) updates only on tick.tick_en.
// - No gated clocks, single clock domain clk_100m.
// - Neighbor-aware: can locally override accel to max brake if too close to car ahead.
//------------------------------------------------------------------------------
module car_node (
  // Tick sideband (clock, reset, 1 kHz enable)
  tick_if.consumer tick,

  // Main Console inputs
  input  amvls_pkg::car_type_e car_type,     // CAR_NORMAL / CAR_TRUCK
  input  amvls_pkg::q8_24_t    a_cmd_in,     // commanded accel (Q8.24)
  input  logic                 cmd_valid_in, // 1 = command for this tick is valid
  input  logic                 hb_in,        // heartbeat from Main (toggle or pulse)

  // Neighbor inputs (car directly ahead)
  input  amvls_pkg::q24_8_t    x_nbr_in,     // neighbor position (Q24.8)
  input  amvls_pkg::q16_16_t   v_nbr_in,     // neighbor velocity (Q16.16) - reserved for future use
  input  logic [7:0]           status_nbr_in,// neighbor status flags (bit 0 = SAFE_HALT, etc.) - reserved for future use

  // Telemetry outputs
  output amvls_pkg::q24_8_t    x_out,        // position (Q24.8)
  output amvls_pkg::q16_16_t   v_out,        // velocity (Q16.16)
  output amvls_pkg::q8_24_t    a_eff_out,    // effective accel actually applied (Q8.24)
  output logic [7:0]           status_flags  // bitfield: basic health / safety
);
  import amvls_pkg::*;

  // Status flag bit positions
  localparam int FLAG_SAFE_HALT   = 0;  // 1 = car is in SAFE_HALT mode
  localparam int FLAG_CMD_MISS    = 1;  // 1 = last tick had cmd_valid_in==0
  localparam int FLAG_HB_LOST     = 2;  // 1 = heartbeat timed out (>1 s)

  // Drag coefficient selection per car type
  q2_30_t k_sel;

  always_comb begin
    case (car_type)
      CAR_TRUCK:  k_sel = K_TRUCK;
      default:    k_sel = K_NORMAL;
    endcase
  end

  // Heartbeat timeout: 1 s at 1 kHz tick
  localparam int unsigned HB_TIMEOUT_TICKS = 1000;

  // Local neighbor-based minimum gap (same 5 m as Main Console MIN_GAP)
  localparam q24_8_t MIN_GAP_LOCAL =
      q24_8_t'(32'sd4 <<< X_F);

  // Heartbeat and safety tracking
  logic        hb_last;
  logic [15:0] hb_miss_cnt;        // enough to count past 1000
  logic        hb_lost;            // heartbeat lost flag
  logic        safe_halt_latched;  // once set, stays until reset

  // Per-tick command-missing flag
  logic        cmd_missing_this_tick;

  // Neighbor gap and local brake request
  amvls_pkg::q24_8_t gap_q24_8;
  logic              need_brake_from_neighbor;

  // Physics state and internal signals
  amvls_pkg::q16_16_t v_int;
  amvls_pkg::q24_8_t  x_int;
  amvls_pkg::q8_24_t  a_drag_mag_int;
  amvls_pkg::q8_24_t  a_eff_int;

  //--------------------------------------------------------------------------
  // Heartbeat monitor and SAFE_HALT latch
  //--------------------------------------------------------------------------
  always_ff @(posedge tick.clk_100m or negedge tick.rst_n) begin
    if (!tick.rst_n) begin
      hb_last               <= 1'b0;
      hb_miss_cnt           <= 16'd0;
      hb_lost               <= 1'b0;
      safe_halt_latched     <= 1'b0;
      cmd_missing_this_tick <= 1'b0;
    end
    else if (tick.tick_en) begin
      // Track whether Main issued a valid command on this tick
      cmd_missing_this_tick <= ~cmd_valid_in;

      // Heartbeat edge detection + timeout
      if (hb_in != hb_last) begin
        hb_last     <= hb_in;
        hb_miss_cnt <= 16'd0;
      end
      else if (!hb_lost) begin
        if (hb_miss_cnt < HB_TIMEOUT_TICKS[15:0])
          hb_miss_cnt <= hb_miss_cnt + 16'd1;
        if (hb_miss_cnt >= HB_TIMEOUT_TICKS[15:0] - 16'd1)
          hb_lost <= 1'b1;
      end

      // SAFE_HALT latching rule:
      // - Once in SAFE_HALT, stay there until reset.
      // - Enter SAFE_HALT if command missing OR heartbeat lost.
      if (!safe_halt_latched) begin
        if (!cmd_valid_in || hb_lost)
          safe_halt_latched <= 1'b1;
      end
    end
  end

  //--------------------------------------------------------------------------
  // Neighbor gap computation and local brake request
  //--------------------------------------------------------------------------
  always_comb begin
    // Distance to car ahead (can be negative if mis-ordered, which we treat as "too close")
    gap_q24_8 = x_nbr_in - x_int;

    // Request local braking when gap is smaller than the minimum allowed spacing
    if (gap_q24_8 < MIN_GAP_LOCAL)
      need_brake_from_neighbor = 1'b1;
    else
      need_brake_from_neighbor = 1'b0;
  end

  //--------------------------------------------------------------------------
  // Command mux: apply Main command, or override to strong brake on faults
  //--------------------------------------------------------------------------
  amvls_pkg::q8_24_t a_cmd_muxed;

  always_comb begin
    if (safe_halt_latched) begin
      // Global SAFE_HALT dominates everything
      a_cmd_muxed = A_CMD_MAX_BRAKE;
    end
    else if (!cmd_valid_in) begin
      // No motion without a valid current-tick command from Main
      a_cmd_muxed = A_CMD_MAX_BRAKE;
    end
    else if (need_brake_from_neighbor) begin
      // Neighbor too close: locally override to strong braking
      a_cmd_muxed = A_CMD_MAX_BRAKE;
    end
    else begin
      // Normal case: follow Main Console command
      a_cmd_muxed = a_cmd_in;
    end
  end

  //--------------------------------------------------------------------------
  // Physics pipeline: drag + acceleration combination + kinematics
  //--------------------------------------------------------------------------
  drag_engine u_drag_engine (
    .v_in       (v_int),
    .k_in       (k_sel),
    .a_drag_mag (a_drag_mag_int)
  );

  accel_combiner u_accel_combiner (
    .a_cmd_in      (a_cmd_muxed),
    .a_drag_mag_in (a_drag_mag_int),
    .v_in          (v_int),
    .a_eff_out     (a_eff_int)
  );

  kinematics_integrator u_kinematics_integrator (
    .tick    (tick),
    .a_eff_in(a_eff_int),
    .v_out   (v_int),
    .x_out   (x_int)
  );

  //--------------------------------------------------------------------------
  // Outputs
  //--------------------------------------------------------------------------
  assign v_out     = v_int;
  assign x_out     = x_int;
  assign a_eff_out = a_eff_int;

  always_comb begin
    status_flags                 = '0;
    status_flags[FLAG_SAFE_HALT] = safe_halt_latched;
    status_flags[FLAG_CMD_MISS]  = cmd_missing_this_tick;
    status_flags[FLAG_HB_LOST]   = hb_lost;
  end

endmodule
