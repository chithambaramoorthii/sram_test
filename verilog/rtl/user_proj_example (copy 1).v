// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
`include "top.v"
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [1:0] wb_sel;
    wire wb_we;
    wire [31:0] wb_addr_i;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire enable_PC;
    wire control_reset;
    wire vclp;
    wire iref0, iref1, iref2, iref3;
    wire in_empty, in_full, weight_empty, weight_full, im_empty, im_full, out_empty, out_full;
    wire [15:0] sa_out;
    
    wire [3:0] wstrb;
    wire [31:0] la_write;

    assign clk = wb_clk_i;
    assign rst = wb_rst_i;
    assign wb_sel = wbs_sel_i[1:0];
    assign wb_we = wbs_we_i;
    assign wb_addr_i = wbs_adr_i;
    assign rdata = wbs_dat_i;
    assign wbs_dat_o = wdata;
    assign enable_PC = la_data_in[0];
    assign control_reset = la_data_in[1];
    assign vclp = io_in[0];
    assign iref0 = io_in[1];
    assign iref1 = io_in[2];
    assign iref2 = io_in[3];
    assign iref3 = io_in[4];
    assign in_empty = la_data_out[29];
    assign in_full = la_data_out[30];
    assign weight_empty = la_data_out[31];
    assign weight_full = la_data_out[32];
    assign im_empty = la_data_out[33];
    assign im_full = la_data_out[34];
    assign out_empty = la_data_out[35];
    assign out_full = la_data_out[36];
    assign sa_out = la_data_out[52:37];

    //WB MI A
   //assign wstrb = wbs_sel_i & {4{wbs_we_i}};
   //assign wbs_dat_o = rdata;
   //assign wdata = wbs_dat_i;

   // IO
   assign io_out = count;
   assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    //assign VSS = vssd1;
    
    

    // LA
    //assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    //assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;


    SRAM_Wrapper_top wrapper(
        `ifdef USE_POWER_PINS
            .VDD(vccd1),
            .VSS(vssa1),
        `endif
        .clk(clk),
        .reset_n(rst),
        .select_buff(wb_sel),
        .enable_PC_IM(enable_PC),
        .cntrl_reset(control_reset),
        .wishbone_buffer_rd_en(wb_we),
        .wishbone_buffer_wr_en(wb_we),
        .wishbone_rw_addr(wb_addr_i),
        .wishbone_buffer_data_in(rdata),
        .VCLP(vclp),
        .Iref0(iref0),
        .Iref1(iref1),
        .Iref2(iref2),
        .Iref3(iref3),
        .IN_buffer_empty(in_empty),
        .IN_buffer_full(in_full),
        .Weight_buffer_empty(weight_empty),
        .Weight_buffer_full(weight_full),
        .IM_buffer_empty(im_empty),
        .IM_buffer_full(im_full),
        .Output_buffer_empty(out_empty),
        .Output_buffer_full(out_full),
        .wishbone_databus_out(wdata),
        .SA_OUT(sa_out)

        );

/*
    (* blackbox *)
    rram_wrapper_16x16 UUT(
        .clk0(clk),
        .VSS(vssd1),
        .WL(WL),
        .BL(BL),
        .SL(SL)
        );

    counter #(
        .BITS(BITS)
    ) counter(
        .clk(clk),
        .reset(rst),
        .ready(wbs_ack_o),
        .valid(valid),
        .rdata(rdata),
        .wdata(wbs_dat_i),
        .wstrb(wstrb),
        .la_write(la_write),
        .la_input(la_data_in[63:32]),
        .count(count)
    ); 

*/
endmodule

module SRAM_Wrapper_top(
`ifdef USE_POWER_PINS
    inout VDD,
    inout VSS,
`endif

//===================================inputs==============================
enable_PC_IM,               // Start program counter IM
clk,                    // Common clock
reset_n,                //wb_rst_i
cntrl_reset, 
wishbone_buffer_rd_en ,     //wbs_we_i=0        // for instruction memory
wishbone_buffer_wr_en ,     //wbs_we_i=1        // for instruction memory      
wishbone_buffer_data_in,            //wbs_dat_i,
wishbone_rw_addr,           //wbs_adr_i
select_buff,            // selects between buffers IM,Input buffer, weight buffer       
Iref0, Iref1, Iref2,Iref3,      // Reference inputs for ADC                 
//======================OUTPUTS=========================    

//Outputs from instruction memory           
IM_buffer_empty,
IM_buffer_full,

//Outputs from weight buffer
Weight_buffer_empty ,
Weight_buffer_full ,

//Outputs from  input buffer
IN_buffer_empty ,
IN_buffer_full ,

//Outputs from Output buffer
Output_buffer_empty ,
Output_buffer_full,
wishbone_databus_out,           //wbs_dat_o
//out_bus_large,
SA_OUT,
VCLP
//VDD,VSS
);

// Global inputs
input clk;
input reset_n;
input [1:0] select_buff;
input enable_PC_IM;             // Start program counter IM
input cntrl_reset;

//Common Inputs for buffers

input wishbone_buffer_rd_en ;           
input wishbone_buffer_wr_en ;
input [31:0]wishbone_rw_addr;  
input [31:0]wishbone_buffer_data_in; 
input VCLP;         
input Iref0;
input Iref1;
input Iref2;
input Iref3;
//inout VDD,VSS;
// SRAM controller outputs
 



// Output for input buffer
output wire IN_buffer_empty ;
output wire IN_buffer_full ;


// Output for weight buffer
output wire  Weight_buffer_empty ;
output wire  Weight_buffer_full ;
            

// Output for Instruction buffer
output wire  IM_buffer_empty ;
output wire  IM_buffer_full ;


// Output signals for Output buffer
output wire Output_buffer_empty ;
output wire Output_buffer_full;
output wire [31:0] wishbone_databus_out;
//output wire [63:0] out_bus_large;



output reg [15:0]  SA_OUT;


reg PRE_SRAM;
reg [15:0]WWL;
reg WE;
reg PRE_VLSA; 
reg PRE_CLSA; 
reg PRE_A;
reg EN; 
reg [15:0]RWL; 
reg [15:0]RWLB;
reg SAEN; 
reg data_ready_signal_output; 
reg writing_finished_signal_output; 
reg busy_signal_output;
reg [15:0]Din;
reg [63:0]IMC_out;




//======================== Top module instantiation =========================================

top DUT_top_control(                  
clk,                    
reset_n,
select_buff,    
enable_PC_IM,               
IMC_out,
cntrl_reset,                // it is output from SRAM and input to the Output Buffer
wishbone_buffer_rd_en ,     //wbs_we_i=0        // for instruction memory
wishbone_buffer_wr_en ,     //wbs_we_i=1        // for instruction memory      
wishbone_rw_addr,           //wbs_adr_i
wishbone_buffer_data_in,            //wbs_dat_i,                        
PRE_SRAM,
WWL, 
WE, 
PRE_VLSA, 
PRE_CLSA, 
PRE_A, 
EN, 
RWL, 
RWLB,
SAEN, 
data_ready_signal_output, 
writing_finished_signal_output, 
busy_signal_output,
Din,                                
IM_buffer_empty,
IM_buffer_full,
Weight_buffer_empty ,
Weight_buffer_full ,
IN_buffer_empty ,
IN_buffer_full ,
Output_buffer_empty ,
Output_buffer_full,
wishbone_databus_out  ,             //wbs_dat_o
//out_bus_large,


);

//=========================================================================================




//============================ SRAM Array Analog part instantiation =======================
Integrated_bitcell_with_dummy_cells  DUT (
`ifdef USE_POWER_PINS
    .VDD(VDD),
    .VSS(VSS),
`endif 
.WWL(WWL),
.RWL(RWL),
.RWLB(RWLB),
.Din(Din),
.WE(WE), 
.PRE_SRAM(PRE_SRAM),
.PRE_VLSA(PRE_VLSA), 
.PRE_CLSA(PRE_CLSA),
.PRE_A(PRE_A), 
.SAEN(SAEN),
.VCLP(VCLP),
.EN(EN),
.Iref0(Iref0), 
.Iref1(Iref1), 
.Iref2(Iref2),
.Iref3(Iref3),
.ADC0_OUT(IMC_out[3:0]),
.ADC1_OUT(IMC_out[7:4]),
.ADC2_OUT(IMC_out[11:8]),
.ADC3_OUT(IMC_out[15:12]),
.ADC4_OUT(IMC_out[19:16]),
.ADC5_OUT(IMC_out[23:20]),
.ADC6_OUT(IMC_out[27:24]),
.ADC7_OUT(IMC_out[31:28]),
.ADC8_OUT(IMC_out[35:32]),
.ADC9_OUT(IMC_out[39:36]),
.ADC10_OUT(IMC_out[43:40]),
.ADC11_OUT(IMC_out[47:44]),
.ADC12_OUT(IMC_out[51:48]),
.ADC13_OUT(IMC_out[55:52]),
.ADC14_OUT(IMC_out[59:56]),
.ADC15_OUT(IMC_out[63:60]),
.SA_OUT(SA_OUT)
);
endmodule



`default_nettype wire
