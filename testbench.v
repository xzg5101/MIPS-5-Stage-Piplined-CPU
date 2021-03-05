`timescale 1ns / 1ps

module IF_EXE_testbench;  

 
    //PC  
    reg clk_tb;   
    reg rst_tb;
    reg update_tb;
    
/////////////////////////////////////////////////////////////////////////////////Wires//////////////////////////////////////////////////////////////////////////////   
    wire [31:0]pc_tb;      

    //instruction memory
    wire [31:0]instr_tb;    
 
    //IF_ID
    wire [5:0] op_tb;
    wire [4:0] rs_tb;
    wire [4:0] rt_tb;
    wire [4:0] rd_tb;
    wire [4:0] sa_tb;
    wire [5:0] func_tb;
    wire [15:0] imm_tb;   
    
    //control unit
    wire    wrt_reg_tb,     mem_to_reg_tb,      wrt_mem_tb,     alu_src_tb,     alu_imm_tb,         reg_d_t_tb;   
    wire [1:0] alu_op_tb;
    wire [3:0] alu_ctrl_tb;

    //write regester mux
    wire [4:0]wrt_reg_mx_tb;
    
    //regester memory
    //wire [4:0] wrt_addr_tb;  = wwrt_reg_mx_tb
    //wire [31:0] wrt_data_tb; = DM_mux_out_tb
    wire [31:0] rd_a_tb;
    wire [31:0] rd_b_tb;   
    
    //sign extend
    wire [31:0] ex_imm_tb;
     
    //ID_EX
    wire ewrt_reg_tb, emem_to_reg_tb, ewrt_mem_tb, ealu_src_tb, ealu_imm_tb, ereg_d_t_tb; 
    wire [1:0] ealu_op_tb; 
    wire [3:0] ealu_ctrl_tb;
    wire [31:0]erd_a_tb, erd_b_tb;
    wire [4:0]ewrt_reg_mx_tb;
    wire [31:0]eex_imm_tb;       
    
    //EX_mux
    wire [31:0] ex_mux_out_tb;
    
    //ALU
    wire [31:0]alu_out_tb;  
    
    //EXE_mem
    wire mmem_to_reg_tb, mwrt_mem_tb, mwrt_reg_tb;
    wire [4:0]mwrt_reg_mx_tb;
    wire [31:0] malu_out_tb, mrd_b_tb; 

    //Data memory            
    wire [31:0] rd_mem_tb;

    //MEM_WB
    wire[31:0]walu_out_tb, wrd_mem_tb;
    wire[4:0] wwrt_reg_mx_tb;
    wire wmem_to_reg_tb;

     //WB mux
     wire[31:0]DM_mux_out_tb;
    

/////////////////////////////////////////////////////////////////////////////////Modules//////////////////////////////////////////////////////////////////////////////
    
    
    PC PC_testbench(clk_tb,rst_tb,update_tb,pc_tb);

    IM IM_testbench(clk_tb,pc_tb,instr_tb);

    IF_ID IF_ID_testbench(clk_tb,instr_tb, op_tb, rs_tb, rt_tb, rd_tb, sa_tb, func_tb, imm_tb);

    CU CU_testbench(clk_tb,op_tb, func_tb, wrt_reg_tb, mem_to_reg_tb, wrt_mem_tb, alu_src_tb, alu_imm_tb, reg_d_t_tb, alu_op_tb, alu_ctrl_tb);

    wrt_reg_mux wrt_reg_mux_testbench(clk_tb, rst_tb, reg_d_t_tb, rd_tb, rt_tb, wrt_reg_mx_tb);

    RM RM_testbench(clk_tb, rst_tb, update_tb, wrt_reg_tb, rs_tb, rt_tb, rd_a_tb, rd_b_tb, wwrt_reg_mx_tb, DM_mux_out_tb);   // wrt_addr_tb =  wwrt_reg_mx_tb;   wrt_data_tb = DM_mux_out_tb;

    SE SE_testbench(clk_tb,imm_tb, ex_imm_tb);
    
    ID_EX ID_EX_testbench( clk_tb,      rst_tb,         wrt_reg_tb,     mem_to_reg_tb,  wrt_mem_tb,     alu_src_tb, 
                           alu_imm_tb,  reg_d_t_tb,     alu_op_tb,      alu_ctrl_tb,    rd_a_tb,        rd_b_tb,        wrt_reg_mx_tb,  ex_imm_tb,
        //output
                           ewrt_reg_tb,    emem_to_reg_tb,             ewrt_mem_tb,    ealu_src_tb,    ealu_imm_tb,    
                           ereg_d_t_tb,    ealu_op_tb, ealu_ctrl_tb,   erd_a_tb,       erd_b_tb,       ewrt_reg_mx_tb, eex_imm_tb      );

    EX_mux EX_mux_testbench (clk_tb, rst_tb, ealu_src_tb, eex_imm_tb, erd_b_tb, ex_mux_out_tb);

    ALU ALU_testbench (clk_tb, rst_tb, erd_a_tb, ex_mux_out_tb, ealu_ctrl_tb, ealu_op_tb, alu_out_tb);

    EXE_mem EXE_mem_testbench(clk_tb,
                emem_to_reg_tb, ewrt_mem_tb, ewrt_reg_mx_tb, alu_out_tb, erd_b_tb, ewrt_reg_tb, 
                mmem_to_reg_tb, mwrt_mem_tb, mwrt_reg_mx_tb, malu_out_tb, mrd_b_tb, mwrt_reg_tb);
                

    DM DM_testbench(clk_tb, mwrt_mem_tb, malu_out_tb, mrd_b_tb, rd_mem_tb); 
    

    
    MEM_WB MEM_WB_testbench(clk_tb,    mmem_to_reg_tb,     malu_out_tb,    rd_mem_tb,  mwrt_reg_tb,    mwrt_reg_mx_tb,
                                        wmem_to_reg_tb,     walu_out_tb,    wrd_mem_tb, wwrt_reg_tb,    wwrt_reg_mx_tb);
                                
    WB_mux WB_mux_testbench(clk_tb,wmem_to_reg_tb, walu_out_tb, wrd_mem_tb, DM_mux_out_tb);
     
     
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    
//initials     
    
    
    initial begin 
         
        rst_tb = 0; 
        clk_tb = 1;
        update_tb = 1;
    end
// loop    
    always begin   
         
         clk_tb = ~clk_tb;
         
         #15;

    end
      
 endmodule
