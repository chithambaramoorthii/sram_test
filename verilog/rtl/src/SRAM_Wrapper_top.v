// Design name : SRAM_top

module SRAM_Wrapper_top(
//===================================inputs==============================
enable_PC_IM, 				// Start program counter IM
clk,					// Common clock
reset_n,				//wb_rst_i
cntrl_reset, 
wishbone_buffer_rd_en ,		//wbs_we_i=0 		// for instruction memory
wishbone_buffer_wr_en ,		//wbs_we_i=1 		// for instruction memory      
wishbone_buffer_data_in,      		//wbs_dat_i,
wishbone_rw_addr,			//wbs_adr_i
select_buff,   			// selects between buffers IM,Input buffer, weight buffer		
Iref0, Iref1, Iref2,Iref3,		// Reference inputs for ADC					
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
wishbone_databus_out,      		//wbs_dat_o
out_bus_large,
SA_OUT,
VCLP,
VDD,VSS
);


// Global inputs
input clk;
input reset_n;
input [1:0] select_buff;
input enable_PC_IM;				// Start program counter IM
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
inout VDD,VSS;
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
output wire [63:0] out_bus_large;



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
cntrl_reset, 				// it is output from SRAM and input to the Output Buffer
wishbone_buffer_rd_en ,		//wbs_we_i=0 		// for instruction memory
wishbone_buffer_wr_en ,		//wbs_we_i=1 		// for instruction memory      
wishbone_rw_addr,			//wbs_adr_i
wishbone_buffer_data_in,      		//wbs_dat_i,					    
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
wishbone_databus_out  ,      		//wbs_dat_o
out_bus_large,


);

//=========================================================================================




//============================ SRAM Array Analog part instantiation =======================

Integrated_bitcell_with_dummy_cells  DUT (
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
.VSS(VSS),
.VDD(VDD),
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
