
`timescale 1ns / 1ps

`include "sram_controller.v"
`include "sync_fifo_32x64.v"
`include "sync_fifo_64x16.v"
`include "sync_fifo_16x16.v"

module top(
//===================================inputs==============================
enable_PC_IM, 				// Start program counter IM
clk,					// Common clock
reset_n,					//wb_rst_i
cntrl_reset, 
wishbone_buffer_rd_en ,		//wbs_we_i=0 		// for instruction memory
wishbone_buffer_wr_en ,		//wbs_we_i=1 		// for instruction memory      
wishbone_buffer_data_in,      		//wbs_dat_i,
wishbone_rw_addr,			//wbs_adr_i
select_buff,   				// selects between buffers IM,Input buffer, weight buffer		
IMC_out,					// it is output from SRAM and input to the Output Buffer
//======================OUTPUTS=========================
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
					
wishbone_databus_out,        		//wbs_dat_o
//out_bus_large,

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
Output_buffer_full
);


// Global inputs
input clk;
input reset_n;
input [1:0] select_buff;
input enable_PC_IM;				// Start program counter IM
reg [5:0]start_PC_IM_address;
input [63:0]IMC_out;				// it is output from SRAM and input to the Output Buffer

// SRAM Controller  inputs
input cntrl_reset;

//Common Inputs for buffers
input wishbone_buffer_rd_en ;			//
input wishbone_buffer_wr_en ;
input [31:0]wishbone_rw_addr;  
input [31:0]wishbone_buffer_data_in; 

// SRAM controller outputs
output wire PRE_SRAM;
output wire [15:0] WWL;
output wire WE;
output wire PRE_VLSA;
output wire PRE_CLSA;
output wire [15:0] PRE_A;
output wire [15:0] EN;
output wire [15:0] RWL;
output wire [15:0] RWLB;
output wire SAEN;
output wire data_ready_signal_output;             //   Ready signal  ----> can be used for PC
output wire writing_finished_signal_output;       //   Writing finished signal  ----> can be used for PC
output wire busy_signal_output;   
output reg [15:0] Din;


			
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
//output wire [63:0]out_bus_large;


//========================================================================================
reg rw;					//rw=1 :for read;  rw=0: for write operation
reg [3:0] address_input;			//address for decoding for memory mode word lines
reg imc_en;				// IMC mode enable
reg mem_en;				// memory mode enable
reg en_dec;				// enable for decoder
wire [15:0]IB_out;
wire [3:0] state_reg_cntrl;				// address for IMC mode word lines to controller input
reg [3:0]state_reg;

reg IM_buffer_wr_en;

reg Input_buffer_wr_en;
reg Input_buffer_rd_en;
reg [3:0]Input_buffer_read_addr;


reg Weight_buffer_wr_en;
reg [3:0]Weight_buffer_read_addr;
reg Weight_buffer_rd_en;
wire [15:0]Weight_buffer_data_out;

reg Output_buffer_wr_en;
reg [3:0]Output_buffer_write_addr;
reg [63:0]Output_buffer_data_in;
wire [31:0]IM_data_out;
reg [31:0]Instruction;
reg [3:0]Temp_x_reg;
reg [3:0]Temp_y_reg;
reg [2:0]IM_state;
wire halt; 
reg halt_IM;
integer count_cycle=0;
reg instr_read;
//==============================================================================


//=================== states definition for selecting buffer====================

parameter
 s0   	=   2'b00,
 s1     =   2'b01,
 s2     =   2'b10;
//==============================================================================

//============== Buffer instantiation for Instruction memory====================

sync_fifo_32x64  DUT_sync_fifo_instruction_memory( 
.clk(clk),
.rst_n(reset_n),
.data_in(wishbone_buffer_data_in),
.rd_en(instr_read),
.wr_en(IM_buffer_wr_en),
.data_out(IM_data_out),
.empty(IM_buffer_empty),
.full(IM_buffer_full) ,
.wr_addr(wishbone_rw_addr[5:0]),
.rd_addr(start_PC_IM_address)
);    

//============================================================================

//=========== Buffer instantiation for Input buffer ==========================
sync_fifo_16x16  DUT_sync_fifo_input_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(wishbone_buffer_data_in[15:0]) ,
.rd_en(Input_buffer_rd_en)    , 		// will come from instruction decoder
.wr_en(Input_buffer_wr_en)    , 		// will come from wishbone 
.data_out(IB_out),			 	// will go to IB_out of sram controller
.empty(IN_buffer_empty)    ,
.full(IN_buffer_full),
.wr_addr(wishbone_rw_addr[3:0]),			//wishbone address
.rd_addr(Input_buffer_read_addr)			// instruction decoder??
);    
//===========================================================================

//=========== Buffer Instantiation for Weight buffer ========================
sync_fifo_16x16  DUT_sync_fifo_Weight_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(wishbone_buffer_data_in[15:0]) ,		// wishbone data in
.rd_en(Weight_buffer_rd_en)   , 	       
.wr_en(Weight_buffer_wr_en)   ,           // will come from wishbone 
.data_out(Weight_buffer_data_out), 			// will go to SRAM data in 
.empty( Weight_buffer_empty)   ,
.full( Weight_buffer_full),
.wr_addr(wishbone_rw_addr[3:0]),			//wishbone address
.rd_addr(Weight_buffer_read_addr)			//write enable for SRAM(WE)
); 

//===================================================================

//=========== Buffer Instantiation for Output buffer ===============
sync_fifo_64x16  DUT_sync_fifo_Output_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(Output_buffer_data_in) ,		// will come from IMC_out(array output)
.rd_en(wishbone_buffer_rd_en)    , 		// will come from wishbone read en or instruction decoder
.wr_en(Output_buffer_wr_en)    ,   		// SAEN
.data_out(wishbone_databus_out), 	       //output_buffer_data_out
.empty( Output_buffer_empty)    ,
.full( Output_buffer_full),
.wr_addr(Output_buffer_write_addr),       	       // will come from Instruction decoder
.rd_addr(wishbone_rw_addr[3:0])		       // may come from Instruction decoder or wishbone address
); 

//===========================================================================

//===========SRAM Controller Instantiation  ========================

sram_controller DUT_sram_control(
.clk(clk), 
.reset(cntrl_reset), 
.rw(rw), 
.address_input(address_input), 
.PRE_SRAM(PRE_SRAM),
.WWL(WWL), 
.WE(WE), 
.PRE_VLSA(PRE_VLSA), 
.data_ready_signal_output(data_ready_signal_output), 
.writing_finished_signal_output(writing_finished_signal_output),
.busy_signal_output(busy_signal_output),
.IB_out(IB_out),
.en_dec(en_dec),
.mem_en(mem_en),
.imc_en(imc_en),
.PRE_CLSA(PRE_CLSA),
.PRE_A(PRE_A),
.EN(EN),
.RWL(RWL),
.RWLB(RWLB),
.SAEN(SAEN), 
.halt(halt),
.state_reg(state_reg_cntrl)
);
  
always @(posedge clk) begin

	if (reset_n)begin
	
		    case(select_buff)
			s0:				//IM buffer write enable
			begin
			IM_buffer_wr_en <= wishbone_buffer_wr_en;
			Input_buffer_wr_en<=0;
			Weight_buffer_wr_en<=0;
			Input_buffer_rd_en<=0;
			Weight_buffer_rd_en<=0;
			end
			
			s1:				//Input buffer write enable
			begin
			IM_buffer_wr_en <= 0;
			Input_buffer_wr_en<=wishbone_buffer_wr_en;
			Weight_buffer_wr_en<=0;
			Input_buffer_rd_en<=0;
			Weight_buffer_rd_en<=0;
			
			end
			
			s2:				//Weight buffer write enable
			begin
			IM_buffer_wr_en <= 0;
			Input_buffer_wr_en<=0;
			Weight_buffer_wr_en<=wishbone_buffer_wr_en;
			Weight_buffer_rd_en<=0;
			Input_buffer_rd_en<=0;
			end
			
		   endcase
	end
end
//=================For control of instruction read================================================

always@(posedge clk) begin
end
//===============================states to be preserved after every change==================================			
always@(posedge clk) begin
IM_state<=IM_data_out[31:29];
instr_read<=(!halt_IM && enable_PC_IM);
state_reg<=state_reg_cntrl;
end
//===========================cases of instruction decoder======================================	
always @(posedge clk)begin
		if(!reset_n) begin
			start_PC_IM_address[5:0]<=6'b000000;
			halt_IM<=1'b0;
		end
		else begin
			if(instr_read) begin
		 	Temp_x_reg<= IM_data_out[7:4];  				//for MAC operation requirement
			Temp_y_reg<= IM_data_out[27:24];				//for MAC operation requirement
			if(halt_IM) begin
			start_PC_IM_address<=start_PC_IM_address;
			end
			else
			begin
			start_PC_IM_address<=start_PC_IM_address + 1'd1;
			end
			end
			case(IM_state)	
			3'b000: 							//writing to SRAM array
			begin
				halt_IM<=1'b1;	
				rw<=1'b0;					//Memory write for rw=0
				mem_en<=1'b1;
				imc_en<=1'b0;
				en_dec<=1'b1;
				address_input[3:0] <= IM_data_out[3:0];		// will go to controller for decoding wordlines
				Weight_buffer_read_addr <= IM_data_out[7:4];  	// data at this address will give data in for SRAM array
				Weight_buffer_rd_en<=1'b1;
				Din[15:0]<= Weight_buffer_data_out[15:0];			//Din is input to SRAM write driver
							
			end
			
			3'b001: 							//Reading from SRAM array
			begin	
				halt_IM<=1'b1;
				address_input[3:0] <= IM_data_out[3:0];		// will go to controller for decoding wordlines
				rw<=1'b1;
				mem_en<=1'b1;
				imc_en<=1'b0;
				en_dec<=1'b1;
			
				//SRAM read data output will go to LA or where?
				
			end
		
			3'b010: 							//IMC operation + mid-buffer write
			begin	
				halt_IM<=1'b1;
				mem_en<=1'b0;
				en_dec<=1'b0;
				rw<=1'b0;
				imc_en<=1'b1;				
				Input_buffer_rd_en<=1'b1;
				Input_buffer_read_addr <= Temp_x_reg;	
					
				if(state_reg==4'b1110)begin
					Output_buffer_wr_en<=1'b1;
					Output_buffer_write_addr<=Temp_y_reg;
					Output_buffer_data_in <= IMC_out;      // will come from SRAM output to output buffer data_in	
					Temp_x_reg<= Temp_x_reg + 1'b1;
					Temp_y_reg<= Temp_y_reg + 1'b1;
					IM_state<=3'b010;
					count_cycle<=count_cycle+1;
					if(count_cycle==(IM_data_out[27:24]-IM_data_out[23:20]))begin 
						halt_IM<=1'b0; //      count will go upto (IM_data_out[28:25]-IM_data_out[24:21])
					end			
				end
				else begin
					IM_state<=3'b010;	
				end
				
				
			end	
			3'b010: begin						//mid buffer to output buffer
				
				
			end		
			endcase
			
		 	
		end

end

endmodule

