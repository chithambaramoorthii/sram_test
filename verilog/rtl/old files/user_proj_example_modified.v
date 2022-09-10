`include	"opcodes.h"

module user_proj_example #(
    parameter INSTRUCTION_SIZE = 32,
    parameter ARRAY_SIZE = 16,
    parameter NUM_WL_ENABLE_MAC_OPS  = 4
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

    //wire [31:0] rdata; 
    //wire [31:0] wdata;
    //wire [BITS-1:0] count;

    //wire valid;
    //wire [3:0] wstrb;
    //wire [31:0] la_write;
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Moorthii
    wire	[INSTRUCTION_SIZE-1:0]	instruction;
    wire [ARRAY_SIZE-1:0] IN0_BL;
    wire [ARRAY_SIZE-1:0] IN1_BL;
    wire [ARRAY_SIZE-1:0] IN0_WL;
    wire [ARRAY_SIZE-1:0] IN1_WL;
    wire [ARRAY_SIZE-1:0] IN0_SL;
    wire [ARRAY_SIZE-1:0] IN1_SL;
    wire ENABLE_WL;
    wire ENABLE_SL;
    wire ENABLE_BL;
    wire [2:0]S_MUX1;
    wire [2:0]S_MUX2;
    wire SEL_MUX1_TO_VSA;
    wire SEL_MUX1_TO_CSA;
    wire SEL_MUX1_TO_ADC;
    wire SEL_MUX2_TO_VSA;
    wire SEL_MUX2_TO_CSA;
    wire SEL_MUX2_TO_ADC;
    wire PRE ;
    wire CLK_EN_ADC1;
    wire CLK_EN_ADC2;
    wire SAEN_CSA1;
    wire SAEN_CSA2;
    
    assign instruction = wbs_dat_i;
    assign la_data_out[15:0] = IN0_BL;
    assign la_data_out[31:16] = IN1_BL;
    assign la_data_out[47:32] = IN0_WL;
    assign la_data_out[63:48] = IN1_WL;
    assign la_data_out[79:64] = IN0_SL;
    assign la_data_out[95:80] = IN1_SL;
    assign la_data_out[96] = ENABLE_WL;
    assign la_data_out[97] = ENABLE_SL;
    assign la_data_out[98] = ENABLE_BL;
    assign la_data_out[101:99] = S_MUX1;
    assign la_data_out[104:102] = S_MUX2;
    assign la_data_out[105] = SEL_MUX1_TO_VSA;
    assign la_data_out[106] = SEL_MUX1_TO_CSA;
    assign la_data_out[107] = SEL_MUX1_TO_ADC;
    assign la_data_out[108] = SEL_MUX2_TO_VSA;
    assign la_data_out[109] = SEL_MUX2_TO_CSA;
    assign la_data_out[110] = SEL_MUX2_TO_ADC;
    assign la_data_out[111] = PRE;
    assign la_data_out[112] = CLK_EN_ADC1;
    assign la_data_out[113] = CLK_EN_ADC2;
    assign la_data_out[114] = SAEN_CSA1;
    assign la_data_out[115] = SAEN_CSA2;

    assign irq = 3'b000;	// Unused
    assign clk = wb_clk_i;
    assign rst = wb_rst_i;


    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////Moorthii
    /*
    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    //assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    assign io_out = count;
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    */


    
    instruction_decoder_RRAM #(
    	.INSTRUCTION_SIZE(INSTRUCTION_SIZE),
    	.ARRAY_SIZE(ARRAY_SIZE),
    	.NUM_WL_ENABLE_MAC_OPS(NUM_WL_ENABLE_MAC_OPS)
    ) uut(
    	.clk(clk),
    	.rst(rst),
    	.instruction(instruction),
    	.IN0_BL(IN0_BL),
	.IN1_BL(IN1_BL),
	.IN0_WL(IN0_WL),
	.IN1_WL(IN1_WL),
	.IN0_SL(IN0_SL),
	.IN1_SL(IN1_SL),
	.ENABLE_WL(ENABLE_WL),
	.ENABLE_SL(ENABLE_SL),
	.ENABLE_BL(ENABLE_BL),
	.S_MUX1(S_MUX1),
	.S_MUX2(S_MUX2),
	.SEL_MUX1_TO_VSA(SEL_MUX1_TO_VSA),
	.SEL_MUX1_TO_CSA(SEL_MUX1_TO_CSA),
	.SEL_MUX1_TO_ADC(SEL_MUX1_TO_ADC),
	.SEL_MUX2_TO_VSA(SEL_MUX2_TO_VSA),
	.SEL_MUX2_TO_CSA(SEL_MUX2_TO_CSA),
	.SEL_MUX2_TO_ADC(SEL_MUX2_TO_ADC),
	.PRE(PRE), 
	.CLK_EN_ADC1(CLK_EN_ADC1),
	.CLK_EN_ADC2(CLK_EN_ADC2),
	.SAEN_CSA1(SAEN_CSA1),
	.SAEN_CSA2(SAEN_CSA2)
    	
    	
    	
    );
/*
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

//	Include definition of the control signals

module instruction_decoder_RRAM #(
    parameter INSTRUCTION_SIZE = 32,
    parameter ARRAY_SIZE = 16,
    parameter NUM_WL_ENABLE_MAC_OPS  = 4
    
)
 (clk,  rst , instruction, 
IN0_BL,
IN1_BL,
IN0_WL,
IN1_WL,
IN0_SL,
IN1_SL,
ENABLE_WL,
ENABLE_SL,
ENABLE_BL,
S_MUX1,
S_MUX2,
SEL_MUX1_TO_VSA,
SEL_MUX1_TO_CSA,
SEL_MUX1_TO_ADC,
SEL_MUX2_TO_VSA,
SEL_MUX2_TO_CSA,
SEL_MUX2_TO_ADC,
PRE, 
CLK_EN_ADC1,
CLK_EN_ADC2,
SAEN_CSA1,
SAEN_CSA2 
);


integer counter_MAC = ARRAY_SIZE/NUM_WL_ENABLE_MAC_OPS ; 


input		clk , rst ;
input	[INSTRUCTION_SIZE-1:0]	instruction;





output reg [ARRAY_SIZE-1:0] IN0_BL;
output reg [ARRAY_SIZE-1:0] IN1_BL;
output reg [ARRAY_SIZE-1:0] IN0_WL;
output reg [ARRAY_SIZE-1:0] IN1_WL;
output reg [ARRAY_SIZE-1:0] IN0_SL;
output reg [ARRAY_SIZE-1:0] IN1_SL;
output reg ENABLE_WL;
output reg ENABLE_SL;
output reg ENABLE_BL;
output reg [2:0]S_MUX1;
output reg [2:0]S_MUX2;
output reg SEL_MUX1_TO_VSA;
output reg SEL_MUX1_TO_CSA;
output reg SEL_MUX1_TO_ADC;
output reg SEL_MUX2_TO_VSA;
output reg SEL_MUX2_TO_CSA;
output reg SEL_MUX2_TO_ADC;
output reg PRE ;
output reg CLK_EN_ADC1;
output reg CLK_EN_ADC2;
output reg SAEN_CSA1;
output reg SAEN_CSA2;

reg  [3:0]COL_ADDR_W;
reg  [3:0]COL_ADDR_R;
reg  [3:0]ROW_ADDR_W;
reg  [3:0]ROW_ADDR_R;


reg is_read_ins;
reg is_write_ins;
reg is_MAC_ins;
reg is_PULSE_T_ins;
reg is_PULSE_V_ins;


reg  [3:0]COL_START_MAC;
reg  [3:0]COL_END_MAC;
reg  [3:0]ROW_START_MAC;
reg  [3:0]ROW_END_MAC;


reg [1:0]T_PULSE_OP_TYPE; 
reg [7:0]T_PULSE_MULTIPLIER; 



reg [1:0]V_PULSE_OP_TYPE; 
reg [1:0]V_PULSE_MUX_SEL; 


reg ENABLE_ARR_OP;
reg  [2:0]temp1;

integer i,j; 

always @(posedge clk   or negedge rst  ) begin 


	if (!rst) begin 

COL_ADDR_W = 0 ;
COL_ADDR_R = 0 ;
ROW_ADDR_W = 0 ;
ROW_ADDR_R = 0 ;

is_read_ins= 0 ;
is_write_ins = 0 ;
is_MAC_ins = 0 ;
is_PULSE_T_ins = 0 ;
is_PULSE_V_ins = 0 ;


COL_START_MAC = 0 ;
COL_END_MAC = 0 ;
ROW_START_MAC = 0 ;
ROW_END_MAC = 0 ;


T_PULSE_OP_TYPE = 0 ; 
T_PULSE_MULTIPLIER = 0 ; 



V_PULSE_OP_TYPE = 0 ; 
V_PULSE_MUX_SEL = 0 ; 

ENABLE_ARR_OP = 0 ; 

	end 

	else begin 
			
case (instruction[31:28])
		`WRITE_RRAM :	// 
                begin
                COL_ADDR_W = instruction[3:0];
                ROW_ADDR_W = instruction[7:4]; 


		ENABLE_ARR_OP = 1 ; 
		is_read_ins= 0 ;
		is_write_ins = 1 ;
		is_MAC_ins = 0 ;
		is_PULSE_T_ins = 0 ;
		is_PULSE_V_ins = 0 ;


		

                end

		`READ_RRAM :	// 
                begin
                COL_ADDR_R = instruction[3:0];
                ROW_ADDR_R = instruction[7:4];
		ENABLE_ARR_OP = 1 ; 

		is_read_ins= 1 ;
		is_write_ins = 0 ;
		is_MAC_ins = 0 ;
		is_PULSE_T_ins = 0 ;
		is_PULSE_V_ins = 0 ;




                end 

		`MAC_OPERATION :	// 
                begin
		
		COL_START_MAC =  instruction[15:12] ;
		COL_END_MAC =    instruction[11:8];
		ROW_START_MAC =  instruction[7:4] ;
		ROW_END_MAC = instruction[3:0] ;
		ENABLE_ARR_OP = 1 ; 

		is_read_ins= 0 ;
		is_write_ins = 0 ;
		is_MAC_ins = 1 ;
		is_PULSE_T_ins = 0 ;
		is_PULSE_V_ins = 0 ;



                end 

		`CONF_T_PULSE_RRAM :	// 
                begin
		

		T_PULSE_OP_TYPE =  instruction[9:8]; 
		T_PULSE_MULTIPLIER = instruction[7:0] ; 
		ENABLE_ARR_OP = 1 ; 


		is_read_ins= 0 ;
		is_write_ins = 0 ;
		is_MAC_ins = 0 ;
		is_PULSE_T_ins = 1 ;
		is_PULSE_V_ins = 0 ;


                end 

		`CONF_V_PULSE_RRAM :	// 
                begin
		

		V_PULSE_OP_TYPE =  instruction[3:2]; 
		V_PULSE_MUX_SEL = instruction[1:0] ; 
		ENABLE_ARR_OP = 1 ; 
		

		is_read_ins= 0 ;
		is_write_ins = 0 ;
		is_MAC_ins = 0 ;
		is_PULSE_T_ins = 0 ;
		is_PULSE_V_ins = 1 ;


                end 



endcase




	end

	end


always @(posedge clk   or negedge rst  ) begin 

	if (!rst) begin 

	IN0_BL = 1 ; 
	IN1_BL = 1 ; 
	IN0_WL = 1 ; 
	IN1_WL = 1 ; 
	IN0_SL = 1 ; 
	IN1_SL = 1 ; 
	ENABLE_WL = 0 ; 
	ENABLE_SL = 0 ; 
	ENABLE_BL = 0 ; 
	S_MUX1 = 0 ; 
	S_MUX2 = 0 ; 
	SEL_MUX1_TO_VSA = 0 ; 
	SEL_MUX1_TO_CSA = 0 ; 
	SEL_MUX1_TO_ADC = 0 ; 
	SEL_MUX2_TO_VSA = 0 ; 
	SEL_MUX2_TO_CSA = 0 ; 
	SEL_MUX2_TO_ADC = 0 ; 
	PRE  = 1 ; 
	CLK_EN_ADC1 = 0 ; 
	CLK_EN_ADC2 = 0 ; 
	SAEN_CSA1 = 0 ; 
	SAEN_CSA2 = 0 ; 

	end 

	else begin 

///////////////////////////////////////////////////////////////////////////////////////////////////		
		if (ENABLE_ARR_OP) begin 

			if (is_write_ins) begin  


				ENABLE_WL = 1 ; 
				ENABLE_SL = 1 ; 
				ENABLE_BL = 1 ; 




				for ( i = 0 ;  i < ARRAY_SIZE ; i = i +1 ) begin  
					if (COL_ADDR_W == i ) begin 
					IN0_BL[COL_ADDR_W] =  0  ; 
					IN1_BL[COL_ADDR_W] =  0  ; 
					IN0_WL[COL_ADDR_W] =  0  ; 
					IN1_WL[COL_ADDR_W] =  0  ; 
					IN0_SL[COL_ADDR_W] =  0  ; 
					IN1_SL[COL_ADDR_W] =  0  ; 
					end else begin  
					IN0_BL[COL_ADDR_W] =  1  ; 
					IN1_BL[COL_ADDR_W] =  1  ; 
					IN0_WL[COL_ADDR_W] =  1  ; 
					IN1_WL[COL_ADDR_W] =  1  ; 
					IN0_SL[COL_ADDR_W] =  1  ; 
					IN1_SL[COL_ADDR_W] =  1  ; 
						end
								           end 


				for ( i = 0 ;  i < ARRAY_SIZE ; i = i +1 ) begin  
					if (ROW_ADDR_W == i ) begin 
					IN0_BL[ROW_ADDR_W] =  0  ; 
					IN1_BL[ROW_ADDR_W] =  0  ; 
					IN0_WL[ROW_ADDR_W] =  0  ; 
					IN1_WL[ROW_ADDR_W] =  0  ; 
					IN0_SL[ROW_ADDR_W] =  0  ; 
					IN1_SL[ROW_ADDR_W] =  0  ; 
					end else begin  
					IN0_BL[ROW_ADDR_W] =  1  ; 
					IN1_BL[ROW_ADDR_W] =  1  ; 
					IN0_WL[ROW_ADDR_W] =  1  ; 
					IN1_WL[ROW_ADDR_W] =  1  ; 
					IN0_SL[ROW_ADDR_W] =  1  ; 
					IN1_SL[ROW_ADDR_W] =  1  ; 
						end
								           end 







									    end 

					end 


/////////////////////////////////////////////////////////////////////////////////////////////////// 


		else if (is_read_ins) begin 


				ENABLE_WL = 0 ; 
				ENABLE_SL = 0 ; 
				ENABLE_BL = 0 ; 

				if (COL_ADDR_R < 8)  begin 
				SEL_MUX1_TO_CSA = 1 ; 
				SEL_MUX2_TO_CSA = 0 ; 

				
				S_MUX1 =  COL_ADDR_R ; 	


			        end else begin 	
				SEL_MUX1_TO_CSA = 0 ; 
				SEL_MUX2_TO_CSA = 1 ; 
				temp1 = COL_ADDR_R - 7 ; 
				S_MUX2 = temp1; 
				end 


				//precharge on +ve cycle and shutoff on
				//negative 
				PRE = 0; 



				for ( i = 0 ;  i < ARRAY_SIZE ; i = i +1 ) begin  
					if (COL_ADDR_R == i ) begin 
					IN0_BL[COL_ADDR_R] =  0  ; 
					IN1_BL[COL_ADDR_R] =  1  ; // 01 is the read voltage 
					IN0_WL[COL_ADDR_R] =  0  ; 
					IN1_WL[COL_ADDR_R] =  0  ; 
					IN0_SL[COL_ADDR_R] =  0  ; 
					IN1_SL[COL_ADDR_R] =  0  ; 
					end else begin  
					IN0_BL[COL_ADDR_R] =  1  ; 
					IN1_BL[COL_ADDR_R] =  1  ; 
					IN0_WL[COL_ADDR_R] =  1  ; 
					IN1_WL[COL_ADDR_R] =  1  ; 
					IN0_SL[COL_ADDR_R] =  1  ; 
					IN1_SL[COL_ADDR_R] =  1  ; 
						end
								           end 


				for ( i = 0 ;  i < ARRAY_SIZE ; i = i +1 ) begin  
					if (ROW_ADDR_R == i ) begin 
					IN0_BL[ROW_ADDR_R] =  0  ; 
					IN1_BL[ROW_ADDR_R] =  0  ; 
					IN0_WL[ROW_ADDR_R] =  0  ; 
					IN1_WL[ROW_ADDR_R] =  0  ; 
					IN0_SL[ROW_ADDR_R] =  0  ; 
					IN1_SL[ROW_ADDR_R] =  0  ; 
					end else begin  
					IN0_BL[ROW_ADDR_R] =  1  ; 
					IN1_BL[ROW_ADDR_R] =  1  ; 
					IN0_WL[ROW_ADDR_R] =  1  ; 
					IN1_WL[ROW_ADDR_R] =  1  ; 
					IN0_SL[ROW_ADDR_R] =  1  ; 
					IN1_SL[ROW_ADDR_R] =  1  ; 
						end
								           end 






		end 


	
				end







		end





always @(negedge  clk   or negedge rst  ) begin 

	if (!rst) begin 

	IN0_BL = 1 ; 
	IN1_BL = 1 ; 
	IN0_WL = 1 ; 
	IN1_WL = 1 ; 
	IN0_SL = 1 ; 
	IN1_SL = 1 ; 
	ENABLE_WL = 0 ; 
	ENABLE_SL = 0 ; 
	ENABLE_BL = 0 ; 
	S_MUX1 = 0 ; 
	S_MUX2 = 0 ; 
	SEL_MUX1_TO_VSA = 0 ; 
	SEL_MUX1_TO_CSA = 0 ; 
	SEL_MUX1_TO_ADC = 0 ; 
	SEL_MUX2_TO_VSA = 0 ; 
	SEL_MUX2_TO_CSA = 0 ; 
	SEL_MUX2_TO_ADC = 0 ; 
	PRE  = 1 ; 
	CLK_EN_ADC1 = 0 ; 
	CLK_EN_ADC2 = 0 ; 
	SAEN_CSA1 = 0 ; 
	SAEN_CSA2 = 0 ; 

	end 

	else begin

		if (is_read_ins) begin 
		PRE = 1;

		
		ENABLE_WL = 1 ; 
		ENABLE_SL = 0 ; 
		ENABLE_BL = 1 ; 
					

		if (COL_ADDR_R < 8)  begin 
		SAEN_CSA1 = 1 ; 
		end else begin 	
		SAEN_CSA2 = 1 ; 
		
		end 


		end 
	end 

end	




endmodule

/*

module counter #(
    parameter BITS = 32
)(
    input clk,
    input reset,
    input valid,
    input [3:0] wstrb,
    input [BITS-1:0] wdata,
    input [BITS-1:0] la_write,
    input [BITS-1:0] la_input,
    output ready,
    output [BITS-1:0] rdata,
    output [BITS-1:0] count
);
    reg ready;
    reg [BITS-1:0] count;
    reg [BITS-1:0] rdata;

    always @(posedge clk) begin
        if (reset) begin
            count <= 0;
            ready <= 0;
        end else begin
            ready <= 1'b0;
            if (~|la_write) begin
                count <= count + 1;
            end
            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= count;
                if (wstrb[0]) count[7:0]   <= wdata[7:0];
                if (wstrb[1]) count[15:8]  <= wdata[15:8];
                if (wstrb[2]) count[23:16] <= wdata[23:16];
                if (wstrb[3]) count[31:24] <= wdata[31:24];
            end else if (|la_write) begin
                count <= la_write & la_input;
            end
        end
    end

endmodule

*/
