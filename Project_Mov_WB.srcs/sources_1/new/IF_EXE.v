`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////////////////////Program Counter///////////////////////////////////////////////////////////////////////////////
module PC(clk, rst, update, pc, stall);
    input clk;
    input rst;
    input update;
    input stall;
    output reg[31:0] pc;
    
    parameter increment = 32'd4;
    initial begin
        pc = 32'd92;
    end
    always @(posedge clk or posedge rst) begin

        if(stall != 1 ) begin
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
		   
			//test codes:                rs    rt    rd
			memory[100] = 32'b000000_00001_00010_00011_00000_100000;    //  add     $3,     $1,     $2
 			memory[104] = 32'b000000_01001_00011_00100_00000_100010;    //  sub     $4,     $9,     $3
 			memory[108] = 32'b000000_00011_01001_00101_00000_100101;    //  or      $5,     $3,     $9
 			memory[112] = 32'b000000_00011_01001_00110_00000_100110;    //  xor     $6,     $3,     $9       
			memory[116] = 32'b000000_00011_01001_00111_00000_100100;    //  and     $7,     $3,     $9

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
    output reg [15:0] imm,
    input stall
    );
    always @(posedge clk)begin
         if(stall == 0)begin
            op <= instr[31:26];
            rs <= instr[25:21];
            rt <= instr[20:16];
            rd <= instr[15:11];
            sa <= instr[10:6];
            func <= instr[5:0];
            imm <= instr [15:0];
         end
   end
endmodule

/////////////////////////////////////////////////////////////////////////////////Control Unit//////////////////////////////////////////////////////////////////////////////
module CU(clk,op, func, rt, rs, ern, mrn, 
                wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t, alu_op, alu_ctrl, stall );    //control unit
    input clk;
    input [5:0]op;
    input [5:0]func;
    input [4:0] rt, rs, ern, mrn;
    
    output reg wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t;
    output reg [1:0] alu_op;
    output reg [3:0] alu_ctrl;
    output reg stall;
    
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
    
    assign add  = (func == 6'b100000)?1:0;
    assign sub  = (func == 6'b100010)?1:0;
    assign exor = (func == 6'b100110)?1:0;
    
    assign alu_add = add | lw |sw;
    assign alu_sub = sub | beq;
    assign alu_and = (func == 6'b100100)?1:0;
    assign alu_or  = (func == 6'b100101)?1:0;
    assign alu_slt = (func == 6'b101010)?1:0;    
    
    //wire [5:0] alu_code = {alu_add, alu_sub, alu_and, alu_or, alu_slt, exor};
    
    initial begin
        stall <= 0;
    end
    
    always@(*)begin
        
        if(stall == 1)begin
            
            
        end
        
        if(rs == ern || rs == mrn || rt == ern || rt == mrn)begin
            stall <= 1;
        end
        else begin
            stall <= 0;
        end
           
     
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
        else if(exor == 1)begin
            alu_ctrl = 4'b1111;
        end
            
        else begin
            alu_ctrl = 4'bXXXX;
        end
    if(stall != 1)begin
        wrt_reg <= wrt_reg_w;
        mem_to_reg <= mem_to_reg_w;
        wrt_mem <= wrt_mem_w;
        alu_src <= alu_src_w;
        alu_imm <= alu_imm_w;
        reg_d_t <= reg_d_t_w;
        alu_op = {rtype,beq};
    end
    end
    
endmodule    
/////////////////////////////////////////////////////////////////////////////////Write Registeter MUX///////////////////////////////////////////////////////////////////////////////
module wrt_reg_mux(clk, rst, reg_d_t, reg_dst, reg_trgt, wrt_reg_mx, stall);
input clk, rst, stall;
input reg_d_t;
input [4:0]reg_dst; 
input [4:0]reg_trgt;
output reg [4:0] wrt_reg_mx;


always@(*)begin
    if(stall == 0)begin
        if(reg_d_t == 1)begin
            wrt_reg_mx = reg_dst;
        end
        else begin
            wrt_reg_mx = reg_trgt;
        end
    end
    else wrt_reg_mx = 'hX;

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
    stall,
    //output
    ewrt_reg,
    emem_to_reg, ewrt_mem, ealu_src, ealu_imm, ereg_d_t, ealu_op, ealu_ctrl,
    erd_a, erd_b,
    ewrt_reg_mx,
    eex_imm,
    ern
    );
    input clk, rst;
   
    
    //input from control unit
    input wrt_reg, mem_to_reg, wrt_mem, alu_src, alu_imm, reg_d_t, stall; 
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
    output reg [4:0]ern;

    
    always @(posedge clk or posedge rst)begin
         
        //if(rst == 0)begin
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
            ern = wrt_reg_mx;
        //end
        if(stall == 1)begin
            ewrt_reg    <= 'bX;
            emem_to_reg <= 'bX;
            ewrt_mem    <= 'bX;
            ealu_src    <= 'bX;
            ealu_imm    <= 'bX;
            ereg_d_t    <= 'bX;
            ealu_op     <= 'bX;
            ealu_ctrl   <= 'bX;
            erd_a       <= 'bX;
            erd_b       <= 'bX;
            ewrt_reg_mx <= 'bX;
            eex_imm     <= 'bX;
            ern         <= 'bX;
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
    
    reg [32:0] temp;
    
    always@(*) begin

        case(ealu_ctrl)                                 //use case
            4'b0000: temp <= erd_a & ex_mux_out;
            4'b0001: temp <= erd_a | ex_mux_out;
            4'b0010: temp <= erd_a + ex_mux_out;
            4'b0110: temp <= erd_a - ex_mux_out;
            4'b0111: temp <= erd_a < ex_mux_out;
            4'b1100: temp <= erd_a ~^ ex_mux_out;
            4'b1111: temp <= erd_a ^ ex_mux_out;
            default: temp <= 'bx;
        endcase
        
        if(ealu_ctrl == 4'b0110 && erd_a < ex_mux_out)
            alu_out = ~temp[31:0]+1;
        else alu_out = temp[31:0];
    end   
endmodule

/////////////////////////////////////////////////////////////////////////////////EXE to Memory//////////////////////////////////////////////////////////////////////////////

module EXE_mem(clk,
                emem_to_reg, ewrt_mem, ewrt_reg_mx, alu_out, erd_b, ewrt_reg, 
                mmem_to_reg, mwrt_mem, mwrt_reg_mx, malu_out, mrd_b, mwrt_reg, mrn);
                
    input clk, emem_to_reg, ewrt_mem, ewrt_reg;
    input [4:0]ewrt_reg_mx;
    input [31:0] alu_out, erd_b;
    
    output reg mmem_to_reg, mwrt_mem, mwrt_reg;
    output reg [4:0]mwrt_reg_mx;
    output reg [31:0] malu_out, mrd_b;
    output reg [4:0]mrn;
    always@(posedge clk)begin
        mmem_to_reg <= emem_to_reg;
        mwrt_mem <= ewrt_mem;
        mwrt_reg_mx <= ewrt_reg_mx;
        malu_out <= alu_out;
        mrd_b <= erd_b;
        mwrt_reg <= ewrt_reg;
        mrn <= ewrt_reg_mx;
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
        
    end
endmodule

/////////////////////////////////////////////////////////////////////////////////Register Memory//////////////////////////////////////////////////////////////////////////////
module RM(clk, rst, updt, wrt_reg, rs, rt, rd_a, rd_b, wrt_addr, wrt_data);
    input clk;
    input rst;
    input updt;
    input wrt_reg;
    
    input [4:0]rs;
    input [4:0]rt;
    
    input [4:0]wrt_addr;
    input [31:0]wrt_data;
    
    output reg [31:0]rd_a;
    output reg [31:0]rd_b;
    

    
    reg [31:0]r[0:31]; 
    integer i;
  
    initial begin
        //for(i = 0; i < 32; i=i+1)
        //          r[i] <= 0;
         r[0] = 32'h00000000;
         r[1] = 32'hA00000AA;
         r[2] = 32'h10000011;
         r[3] = 32'h20000022;
         r[4] = 32'h30000033;
         r[5] = 32'h40000044;
         r[6] = 32'h50000055;
         r[7] = 32'h60000066;
         r[8] = 32'h70000077;
         r[9] = 32'h80000088;
         r[10]= 32'h90000099;
                 
    end
    always @(*)begin
        

        if((wrt_reg == 1))begin
            r[wrt_addr] <= wrt_data;
        end 

        rd_a <= r[rs];
        rd_b <= r[rt];

    end   
    
endmodule
