
module main_console_full_for_N_Car_Node #(
  parameter int unsigned N_CARS = 1
)(
  tick_if.consumer         tick,

  input  amvls_pkg::q24_8_t   x_from_car      [N_CARS],
  input  amvls_pkg::q16_16_t  v_from_car      [N_CARS],
  input  amvls_pkg::q8_24_t   a_eff_from_car  [N_CARS],
  input  logic [7:0]          status_from_car [N_CARS],

  output amvls_pkg::q8_24_t   a_cmd_to_car    [N_CARS],
  output logic                cmd_valid_to_car[N_CARS],
  output logic                hb_to_car,

  output amvls_pkg::tick_id_t tick_id_out
);
  import amvls_pkg::*;

  logic hb_reg;

  localparam int unsigned CAR_IDX_W = (N_CARS <= 1) ? 1 : $clog2(N_CARS);
  logic                 processing;             // 1 = walking cars this tick
  logic [CAR_IDX_W-1:0] car_idx;               // current car index

  q8_24_t a_cmd_reg    [N_CARS];
  logic   cmd_valid_reg[N_CARS];

  localparam q16_16_t V_TARGET =
      q16_16_t'(32'sd2000 <<< V_F);

  localparam q16_16_t V_ERR_DEADBAND =
      q16_16_t'(32'sd1 <<< (V_F-4));

  localparam q24_8_t MIN_GAP =
      q24_8_t'(32'sd5 <<< X_F);

  q8_24_t a_cmd_calc;

  always_comb begin
    a_cmd_calc = a_cmd_reg[car_idx];

    if (processing) begin
      if (v_from_car[car_idx] < (V_TARGET - V_ERR_DEADBAND)) begin
        a_cmd_calc = A_CMD_MAX_ACCEL;
      end
      else if (v_from_car[car_idx] > (V_TARGET + V_ERR_DEADBAND)) begin
        a_cmd_calc = A_CMD_MAX_BRAKE;
      end
      else begin
        a_cmd_calc = '0;
      end

      if ((N_CARS > 1) && (car_idx != {CAR_IDX_W{1'b0}})) begin
        int unsigned prev_idx;
        prev_idx = car_idx - 1;
        if ((x_from_car[prev_idx] - x_from_car[car_idx]) < MIN_GAP) begin
          a_cmd_calc = A_CMD_MAX_BRAKE;
        end
      end
    end
  end

  always_ff @(posedge tick.clk_100m or negedge tick.rst_n) begin
    if (!tick.rst_n) begin
      hb_reg     <= 1'b0;
      processing <= 1'b0;
      car_idx    <= '0;

      for (int j = 0; j < N_CARS; j++) begin
        a_cmd_reg[j]     <= '0;
        cmd_valid_reg[j] <= 1'b1;
      end
    end
    else begin
      if (tick.tick_en) begin
        hb_reg     <= ~hb_reg;
        processing <= 1'b1;
        car_idx    <= '0;
      end

      if (processing) begin
        a_cmd_reg[car_idx]     <= a_cmd_calc;
        cmd_valid_reg[car_idx] <= 1'b1;

        if (car_idx == (N_CARS-1)) begin
          processing <= 1'b0;
        end
        else begin
          car_idx <= car_idx + 1'b1;
        end
      end
    end
  end
  
  assign hb_to_car   = hb_reg;
  assign tick_id_out = tick.tick_id;

  assign a_cmd_to_car     = a_cmd_reg;
  assign cmd_valid_to_car = cmd_valid_reg;

endmodule

