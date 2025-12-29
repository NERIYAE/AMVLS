
module amvls_top #(
  parameter int unsigned N_CARS = 5  
)(
  input  logic                 clk_100m,
  input  logic                 rst_n,

  output amvls_pkg::q24_8_t    car_x      [N_CARS],
  output amvls_pkg::q16_16_t   car_v      [N_CARS],
  output amvls_pkg::q8_24_t    car_a_eff  [N_CARS],
  output logic [7:0]           car_status [N_CARS],
  output amvls_pkg::tick_id_t  tick_id_out
);
  import amvls_pkg::*;

  tick_if tick (
    .clk_100m (clk_100m),
    .rst_n    (rst_n)
  );

  amvls_tick_gen u_tick_gen (
    .tick (tick)  
  );

  q24_8_t     x_from_car        [N_CARS];
  q16_16_t    v_from_car        [N_CARS];
  q8_24_t     a_eff_from_car    [N_CARS];
  logic [7:0] status_from_car   [N_CARS];

  q8_24_t   a_cmd_to_car     [N_CARS];
  logic     cmd_valid_to_car [N_CARS];
  logic     hb_to_car;
  tick_id_t tick_id_mc;


  q24_8_t     x_nbr_to_car        [N_CARS];
  q16_16_t    v_nbr_to_car        [N_CARS];
  logic [7:0] status_nbr_to_car   [N_CARS];

  localparam q24_8_t X_FRONT_NEIGHBOR_Q24_8 =
      q24_8_t'(32'sd1000 <<< X_F);

  assign x_nbr_to_car[0]      = X_FRONT_NEIGHBOR_Q24_8;
  assign v_nbr_to_car[0]      = '0;
  assign status_nbr_to_car[0] = 8'h00;

  genvar j;
  generate
    for (j = 1; j < N_CARS; j++) begin : gen_neighbor_map
      assign x_nbr_to_car[j]      = x_from_car[j-1];
      assign v_nbr_to_car[j]      = v_from_car[j-1];
      assign status_nbr_to_car[j] = status_from_car[j-1];
    end
  endgenerate


  main_console_full_for_N_Car_Node #(
    .N_CARS (N_CARS)
  ) u_main_console (
    .tick            (tick),

    .x_from_car      (x_from_car),
    .v_from_car      (v_from_car),
    .a_eff_from_car  (a_eff_from_car),
    .status_from_car (status_from_car),

    .a_cmd_to_car     (a_cmd_to_car),
    .cmd_valid_to_car (cmd_valid_to_car),
    .hb_to_car        (hb_to_car),
    .tick_id_out      (tick_id_mc)
  );

  genvar i;
  generate
    for (i = 0; i < N_CARS; i++) begin : gen_cars
      car_node u_car (
        .tick        (tick),
        .car_type    (CAR_NORMAL),
        .a_cmd_in     (a_cmd_to_car[i]),
        .cmd_valid_in (cmd_valid_to_car[i]),
        .hb_in        (hb_to_car),
        .x_nbr_in      (x_nbr_to_car[i]),
        .v_nbr_in      (v_nbr_to_car[i]),
        .status_nbr_in (status_nbr_to_car[i]),
        .x_out        (x_from_car[i]),
        .v_out        (v_from_car[i]),
        .a_eff_out    (a_eff_from_car[i]),
        .status_flags (status_from_car[i])
      );
    end
  endgenerate

  assign car_x      = x_from_car;
  assign car_v      = v_from_car;
  assign car_a_eff  = a_eff_from_car;
  assign car_status = status_from_car;

  assign tick_id_out = tick_id_mc;

endmodule
