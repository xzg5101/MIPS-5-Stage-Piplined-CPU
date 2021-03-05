`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////////////////////Program Counter///////////////////////////////////////////////////////////////////////////////
module PC(clk, rst, update, pc);
    input clk;
    input rst;
    input update;
    output reg[31:0] pc;
    
    parameter increment = 32'd4;
    initial begin
        pc = 32'd0;
    end
    always @(posedge clk or posedge rst) begin
        if (rst)begin
            pc <= 0;
        end
        else begin
            pc <= pc + increment; 
        end    
    end
endmodule
/////////////////////////////////////////////////////////////////////////////////Instruction Memory///////////////////////////////////////////////////////////////////////////////
module IM(clk,pc,instr);
	
	input clk;
	input[31:0] pc;
	output reg[31:0] instr;
	
	reg [31:0] memory [0:255];
	
	integer addr;
	
	initial
		begin
		    //                  op      rs    rt       imm               op   rt      sh  rd
			//memory[100] = 32'b100011_00001_00010_0000000000000000;   //lw   $v0,    00($at)   100011_00001_00010_0000000000000000
			//memory[104] = 32'b100011_00001_00010_0000000000000100;   //lw   $v0,    04($at)   100011_00001_00010_0000000000000100
			memory[100] = 32'b100011_00001_00010_0000000000000000;//lw   $2,     00($1)     100011_00001_00010_0000000000000000    # $2 ??memory[$1+00]; load x[0]
 			memory[104] = 32'b100011_00001_00011_0000000000000100;//lw   $3,     04($1)     100011_00001_00011_0000000000000100    # $3 ??memory[$1+04]; load x[1]
 			memory[108] = 32'b100011_00001_00100_0000000000001000;//lw   $4,     08($1)     100011_00001_00100_0000000000000100     # $4??memory[$1+08]; load x[2]
 			memory[112] = 32'b100011_00001_00101_0000000000001100;//lw   $5,     12($1)# $5??memory[$1+12]; load x[3]
 			
 			//                  add     $2    $10    $6         
			memory[116] = 32'b000000_00010_01010_00110_00000_100000;//add  $6,     $2,     $10     
			
			//test codes:
			//memory[108] = 32'b100011_00001_00010_1000000000000100;   //lw $v0, 04($at)   100011_00001_00010_0000000000000100
			//memory[112] = 32'b100011_00011_01010_1100000100000100;   //lw $v0, 04($at)   100011_00001_00010_0000000000000100
			//                  op      rs     rt     rd   shamt   func
 			//memory[116] = 32'b000000_01001_00010_01000_00000_100000;   //add $t0, $t0, $t1   000000_01001_01010_01000_00000_100000

		end
  
	always@(*) begin
		addr = pc;
		instr = memory[addr];
	end
endmodule

/////////////////////////////////////////////////////////////////////////////////IF_ID///////////////////////////////////////////////////////////////////////////////

//IF_ID using registers
//consum one extra clock cycle
module IF_ID(input clk, 
    input [31:0] instr,
    output reg [5:0] op,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] rd,
    output reg [4:0] sa, 
    output reg [5:0] func,  
    output reg [15:0] imm
    );
    always @(posedge clk)begin
         op <= instr[31:26];
         rs <= instr[25:21];
         rt <= instr[20:16];
         rd <= instr[15:11];
         sa <= instr[10:6];
         func <= instr[5:0];
         imm <= instr [15:0];
   end
endmodule


//IF_ID using wire
/*
module IF_ID(input clk, 
    input [31:0] instr,
    output [5:0] op,
    output [4:0] rs,
    output [4:0] rt,
    output [4:0] rd,
    output [4:0] sa, 
    output [5:0] func,  
    output [15:0] imm
    );
        assign op = instr[31:26];
        assign rs = instr[25:21];
        assign rt = instr[20:16];
        assign rd = instr[15:11];
        assign sa = instr[10:6];
        assign func = instr[5:0];
        assign imm = instr [15:0];

endmodule
*/
/////////////////////////////////////////////////////////////////////////////////Control Unit//////////////////////////////////////////////////////////////////////////////
module CU(clk,op, func, wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t, alu_op, alu_ctrl);    //control unit
    input clk;
    input [5:0]op;
    input [5:0]func;
    
    output reg wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t;
    output reg [1:0] alu_op;
    output reg [3:0] alu_ctrl;
    
    //identify the instruction types:
    assign rtype= (op == 6'b000000)?1:0;    //rtype = 1 if op = 000000  
    assign lw   = (op == 6'b100011)?1:0;    //lw    = 1 if op = 100011
    assign sw   = (op == 6'b101011)?1:0;    //sw    = 1 if op = 101011
    assign beq  = (op == 6'b000100)?1:0;    //beq   = 1 if op = 000100
    assign j    = (op == 6'b000010)?1:0;    //j     = 1 if op = 000010
    
    assign addi = (op == 6'b001000)?1:0;    //addi  = 1 if op = 001000
    assign andi = (op == 6'b001100)?1:0;    //andi  = 1 if op = 001100
    assign ori  = (op == 6'b001101)?1:0;    //ori   = 1 if op = 001101
    assign xori = (op == 6'b001110)?1:0;    //xori  = 1 if op = 001110

    //associate the read and write with control outputs:
    assign alu_imm_w = addi | andi | ori | xori;       //imm if i 
    assign reg_d_t_w = rtype;
    assign wrt_reg_w = rtype|lw;
    assign mem_to_reg_w = lw;
    assign wrt_mem_w = sw;
    assign alu_src_w = sw|lw;
    
    assign add = (func == 6'b100000)?1:0;
    assign sub = (func == 6'b100010)?1:0;
    
    assign alu_add = add | lw |sw;
    assign alu_sub = sub | beq;
    assign alu_and = (func == 6'b100100)?1:0;
    assign alu_or  = (func == 6'b100101)?1:0;
    assign alu_slt = (func == 6'b101010)?1:0;    
    
    always@(*)begin
        
        if(alu_add == 1)begin
            alu_ctrl = 4'b0010;
        end 
        else if(alu_sub == 1)begin
            alu_ctrl = 4'b0110;
        end 
        else if(alu_and == 1)begin
            alu_ctrl = 4'b0000;
        end 
        else if(alu_or == 1)begin
            alu_ctrl = 4'b0001;
        end 
        else if(alu_slt == 1)begin
            alu_ctrl = 4'b0111;
        end
        else begin
            alu_ctrl = 4'bXXXX;
        end
        
        
         
        wrt_reg <= wrt_reg_w;
        mem_to_reg <= mem_to_reg_w;
        wrt_mem <= wrt_mem_w;
        alu_src <= alu_src_w;
        alu_imm <= alu_imm_w;
        reg_d_t <= reg_d_t_w;
        alu_op = {rtype,beq};
        
    end
endmodule    
/////////////////////////////////////////////////////////////////////////////////Write Registeter MUX///////////////////////////////////////////////////////////////////////////////
module wrt_reg_mux(clk, rst, reg_d_t, reg_dst, reg_trgt, wrt_reg_mx);
input clk, rst;
input reg_d_t;
input [4:0]reg_dst; 
input [4:0]reg_trgt;
output reg [4:0] wrt_reg_mx;


always@(*)begin
    if(rst == 0)begin        
        if(reg_d_t == 1)begin
            wrt_reg_mx <= reg_dst;
        end
        else begin
            wrt_reg_mx <= reg_trgt;
        end
    end
    else begin
        wrt_reg_mx <= 5'b00000; 
    end
    
end
endmodule
/////////////////////////////////////////////////////////////////////////////////Register Memory//////////////////////////////////////////////////////////////////////////////
module RM(clk, rst, updt, wrt_reg, addr_a, addr_b, rd_a, rd_b, wrt_addr, wrt_data);
    input clk;
    input rst;
    input updt;
    input wrt_reg;
    
    input [4:0]addr_a;
    input [4:0]addr_b;
    
    input [4:0]wrt_addr;
    input [31:0]wrt_data;
    
    output reg [31:0]rd_a;
    output reg [31:0]rd_b;
    
    reg [31:0]r[1:31]; 
    integer i;
  
    initial begin
        for(i = 0; i < 32; i=i+1)
                    r[i] <= 0;
    end
    
    always @(*)begin
        
        if(rst == 1)begin
            for(i = 0; i < 32; i=i+1)
                    r[i] <= 0;
        end
        
        else if(rst == 0) begin
            if(updt == 1)begin
                if((wrt_reg == 1))begin
                    r[wrt_addr] <= wrt_data;
                end 

                    rd_a <= r[addr_a];
                    rd_b <= r[addr_b];

            end
        end
    end
endmodule

/////////////////////////////////////////////////////////////////////////////////Sign Extend///////////////////////////////////////////////////////////////////////////////
module SE(clk, imm, ex_imm);
input clk;
input [15:0]imm;
output reg [31:0]ex_imm;

always @(*)begin
    ex_imm[15:0] <= imm;
    if(imm[15] == 1)begin
        ex_imm[31:16] <= 16'b1111_1111_1111_1111;
    end
    else if(imm[15] == 0) begin
        ex_imm[31:16] <= 16'b0000_0000_0000_0000;
    end
    
    else  ex_imm[31:16] <= 16'bx;
end
endmodule

/////////////////////////////////////////////////////////////////////////////////ID EX///////////////////////////////////////////////////////////////////////////////
module ID_EX(
    //input 
    clk, rst, 
    wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t, alu_op, alu_ctrl,
    rd_a, rd_b,
    wrt_reg_mx,
    ex_imm,
    
    
    //output
    
    ewrt_reg,
    emem_to_reg, ewrt_mem, ealu_src, ealu_imm, ereg_d_t, ealu_op, ealu_ctrl,
    erd_a, erd_b,
    ewrt_reg_mx,
    eex_imm
    );
    input clk, rst;
   
    
    //input from control unit
    input wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t; 
    input[1:0]alu_op; 
    input [3:0] alu_ctrl;
    
    //input from regester memory
    input [31:0]rd_a, rd_b;
    
    //input from write register mux
    input [4:0]wrt_reg_mx;
    
    //input from sign extend
    input [31:0]ex_imm;
    
    
    //output
   
    output reg ewrt_reg,    //write register
            emem_to_reg,     //memory to register
            ewrt_mem,        //write memory
            ealu_src,        //
            ealu_imm,        //alu immediate
            ereg_d_t;        //register rd or rt
    output reg [1:0] ealu_op; 
    output reg [3:0] ealu_ctrl;
    output reg [31:0]erd_a, erd_b;  //register memory out out
    output reg [4:0] ewrt_reg_mx;   //write register memory mux
    output reg [31:0]eex_imm;       //extended immediate
    
    always @(posedge clk or posedge rst)begin
        if(rst == 0)begin
            ewrt_reg <= wrt_reg;
            emem_to_reg <= mem_to_reg;
            ewrt_mem <= wrt_mem;
            ealu_src <= alu_src;
            ealu_imm <= alu_imm;
            ereg_d_t <= reg_d_t;
            ealu_op <= alu_op;
            ealu_ctrl <= alu_ctrl;
            erd_a <= rd_a;
            erd_b <= rd_b;
            ewrt_reg_mx <= wrt_reg_mx;
            eex_imm <= ex_imm;
        end
        else if(rst == 1) begin
            ewrt_reg <= 'b0;
            emem_to_reg <= 'b0;
            ewrt_mem <= 'b0;
            ealu_src <= 'b0;
            ealu_imm <= 'b0;
            ereg_d_t <= 'b0;
            ealu_op <= 'b0;
            ealu_ctrl <= 'b0;
            erd_a <= 'b0;
            erd_b <= 'b0;
            ewrt_reg_mx <= 'b0;
            eex_imm <= 'b0;
        
        end 
    end
    
endmodule 

/////////////////////////////////////////////////////////////////////////////////EX mux///////////////////////////////////////////////////////////////////////////////

module EX_mux(clk, rst, ealu_src, eex_imm, erd_b, ex_mux_out);
input clk, rst;
input ealu_src;
input [31:0]eex_imm;
input [31:0]erd_b;

output reg [31:0] ex_mux_out;

    always@* begin
        ex_mux_out <=(ealu_src == 1)? eex_imm : erd_b;
    end
endmodule

/////////////////////////////////////////////////////////////////////////////////ALU///////////////////////////////////////////////////////////////////////////////

module ALU(clk, rst, erd_a, ex_mux_out, ealu_ctrl, ealu_op, alu_out);

    input clk, rst;
    input [31:0] erd_a, ex_mux_out;
    input [3:0] ealu_ctrl;
    input [1:0] ealu_op;
    
    output reg [31:0] alu_out;
    
    always@* begin

        case(ealu_ctrl)                                 //use case
            4'b0000: alu_out <= erd_a & ex_mux_out;
            4'b0001: alu_out <= erd_a | ex_mux_out;
            4'b0010: alu_out <= erd_a + ex_mux_out;
            4'b0110: alu_out <= erd_a - ex_mux_out;
            4'b0111: alu_out <= erd_a < ex_mux_out;
            4'b1100: alu_out <= erd_a ~^ ex_mux_out;
            default: alu_out <= 'bx;
        endcase
    end   
endmodule

/////////////////////////////////////////////////////////////////////////////////EXE to Memory//////////////////////////////////////////////////////////////////////////////

module EXE_mem(clk,
                emem_to_reg, ewrt_mem, ewrt_reg_mx, alu_out, erd_b, ewrt_reg, 
                mmem_to_reg, mwrt_mem, mwrt_reg_mx, malu_out, mrd_b, mwrt_reg);
                
    input clk, emem_to_reg, ewrt_mem, ewrt_reg;
    input [4:0]ewrt_reg_mx;
    input [31:0] alu_out, erd_b;
    
    output reg mmem_to_reg, mwrt_mem, mwrt_reg;
    output reg [4:0]mwrt_reg_mx;
    output reg [31:0] malu_out, mrd_b;

    always@(posedge clk)begin
        mmem_to_reg <= emem_to_reg;
        mwrt_mem <= ewrt_mem;
        mwrt_reg_mx <= ewrt_reg_mx;
        malu_out <= alu_out;
        mrd_b <= erd_b;
        mwrt_reg <= ewrt_reg;
    end

endmodule




/////////////////////////////////////////////////////////////////////////////////Data Memory//////////////////////////////////////////////////////////////////////////////

module DM(clk, mwrt_mem, malu_out, mrd_b, rd_mem);
    input clk, mwrt_mem;
    input [31:0]mrd_b, malu_out;
    output reg [31:0] rd_mem;
    reg [31:0] mem_data [0:64];
    
    wire [31:0] mem_addr = malu_out;
    wire [31:0] mem_wrt_data = mrd_b;
    integer i;
    initial begin
                                                                                    //set as zero
        for(i = 40; i < 64; i=i+4)
            mem_data[i] <= 0;
        
        mem_data[0]   = 32'hA00000AA; 
        mem_data[4]   = 32'h10000011; 
		mem_data[8]   = 32'h20000022; 
		mem_data[12]  = 32'h30000033; 
		mem_data[16]  = 32'h40000044; 
		mem_data[20]  = 32'h50000055; 
		mem_data[24]  = 32'h60000066; 
		mem_data[28]  = 32'h70000077; 
		mem_data[32]  = 32'h80000088; 
		mem_data[36]  = 32'h90000099; 
	end
	always @(*)begin
	
	   rd_mem <= mem_data[mem_addr];
	   
	   if(mwrt_mem == 1'd1)begin
	       mem_data[mem_addr] = mrd_b;
	   end
	end
endmodule



/////////////////////////////////////////////////////////////////////////////////MEM WB//////////////////////////////////////////////////////////////////////////////

module MEM_WB(clk,                                                              //pass values with clock
                mmem_to_reg, malu_out, rd_mem, mwrt_reg, mwrt_reg_mx,
               wmem_to_reg, walu_out, wrd_mem, wwrt_reg, wwrt_reg_mx);
    
    input clk, mmem_to_reg, mwrt_reg;
    input [4:0] mwrt_reg_mx;
    input [31:0]malu_out;
    input [31:0] rd_mem;
    
    output reg wmem_to_reg, wwrt_reg;
    output reg [31:0] walu_out;
    output reg [31:0] wrd_mem;
    output reg [4:0]  wwrt_reg_mx;
       
    always@(posedge clk)begin
        wmem_to_reg <= mmem_to_reg;
        wrd_mem <= rd_mem;
        walu_out <= malu_out;
        wwrt_reg <= mwrt_reg;
        wwrt_reg_mx <= mwrt_reg_mx;
    end
endmodule

/////////////////////////////////////////////////////////////////////////////////Data Memory Mux//////////////////////////////////////////////////////////////////////////////

module WB_mux(clk,wmem_to_reg, walu_out, wrd_mem, DM_mux_out);  //last mux
    input clk, wmem_to_reg;
    input [31:0]walu_out, wrd_mem;
    output reg [31:0]DM_mux_out;         //this give the data to write in the register memory
    always @* begin
        
        DM_mux_out <=(wmem_to_reg == 1'b1)? wrd_mem : walu_out;
        
        /*
        if(wmem_to_reg == 1'b1)begin
            DM_mux_out <= wrd_mem;
        end
        else if (wmem_to_reg == 1'b0)begin
            DM_mux_out <= walu_out;
        end
        else
            DM_mux_out <= 32'bX;   
        */
    end
endmodule





