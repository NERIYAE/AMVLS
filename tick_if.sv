
interface tick_if (
  input  logic clk_100m,  // global 100 MHz fabric clock
  input  logic rst_n      // active-low global reset
);
  import amvls_pkg::*;


  logic      tick_en;
  tick_id_t  tick_id;


  modport producer (
    input  clk_100m,
    input  rst_n,
    output tick_en,
    output tick_id
  );

  modport consumer (
    input clk_100m,
    input rst_n,
    input tick_en,
    input tick_id
  );

endinterface
