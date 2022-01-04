module complete();
reg[6:0] PC;
reg[31:0] PC1;  
reg clk, add_checker;
wire[2:0] ALUctl; 
wire I_type, mem_read, mem_write, memToRegister, bne_checker, beq_checker, zerobit; 
wire [31:0] out, extended, data1, data2, ALUOut, Data, add_factor;
reg[4:0] write_address;
reg[31:0] inalu, write_data;
always@(*) 
begin
	assign write_address = (I_type==1) ? out[20:16] : out[15:11];
	assign inalu = (I_type==1) ? extended : data2;
	assign write_data = (memToRegister==1) ? Data : ALUOut;
	assign add_checker = (~zerobit&&bne_checker)||(zerobit&&beq_checker); 
end
instructionmem im_00(PC, clk, out);
register_file rf_00(out[25:21], out[20:16], write_address, write_data, write_enable, data1, data2);
ctrl ct_00(out[5:0], out[31:26], enable, ALUctl, I_type, write_enable, mem_read, mem_write, memToRegister, bne_checker, beq_checker, jr_checker, jt_checker);
alu alu_00(ALUctl, data1, inalu, ALUOut, enable, zerobit);
main_memory mainmemory_00(ALUOut, data2, ALUOut, mem_read, mem_write, Data);
sign_extension signe_00(extended, out[15:0]);
multiplier aaaa00(extended, add_factor);
initial 
begin
PC=0;
clk=0;
forever #10 clk=~clk;
end

//Program Counter code
always@(posedge clk)
begin
	PC1= PC+4;
	if(PC != 7'b1111100)
	begin 
		if(add_checker==1) PC = PC1+add_factor;
		else if(jr_checker==1) PC = data1;
		else if(jt_checker==1) PC={PC1[31:29],out[25:0],2'b00}; 
		else PC=PC1;
	end
end
endmodule 


//sign extension module 
module sign_extension(output reg [31:0] extended, input[15:0] extend);
always@(*) begin
assign extended = {extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],
                   extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],extend[15],extend[15:0]};
end
endmodule 

// instruction memory module 
module instructionmem(input[6:0] PC, input clk, output reg[31:0] out);
integer x;
reg[7:0] mem[127:0];

always@(posedge clk)
begin
	out = {mem[PC],mem[PC+1],mem[PC+2],mem[PC+3]};
end
initial 
begin 
	/*for(x=0;x<128;x=x+1) 
	begin
		mem[x]=0;
	end	
	{mem[0],mem[1],mem[2], mem[3]}<=32'b000000_00001_00010_00101_00000_100000; // add $5,$1,$2
	{mem[4],mem[5],mem[6], mem[7]}<=32'b100011_00001_00010_0010100000100000; */
	$readmemh("instruction_mem.txt", mem, 0, 7); 
end
endmodule 

//multiplier module 
module multiplier(input[31:0] m_in, output reg[31:0] m_out);
always@(*)
begin
	assign m_out = {m_in[29:0],2'b00};
end
endmodule 

// register file module 
module register_file(input[4:0] read_addr1, read_addr2, write_address, input[31:0] write_data, input enable, output reg[31:0] data1, data2);
reg[31:0] mem1[31:0];
integer x;
always@(read_addr1, read_addr2)
begin
	assign data1=mem1[read_addr1];
	assign data2=mem1[read_addr2];
end
always@(write_data, write_address, enable) 
begin
	if(enable==1) 
	mem1[write_address]=write_data;
end      	 
initial 
begin 
	for(x=0;x<32;x=x+1) 
	begin
		mem1[x]=0;
	end	
	
	mem1[1]<=32'b00000000000000000000000000000001; //a=1
  	mem1[2]<=32'b00000000000000000000000000000010; //b=2
	//mem1[4]<=32'b00000000000000000000000000010110; //22
	//mem1[20]<=32'b00000000000000001111111111111111; // for masking */
end
endmodule


// main memory  
module main_memory(input[31:0] read_address, write_data, write_address, input mem_read, mem_write, output reg[31:0] Data);
reg[31:0] mem_main[31:0];
integer x;
always@(*) 
begin
	if(mem_read==1) Data=mem_main[read_address];
	else if(mem_write==1) mem_main[write_address]=write_data;
end      	 
initial 
begin 
	for(x=0;x<32;x=x+1) 
	begin
		mem_main[x]=0;
	end	
	mem_main[0]=32'b00000000000000000000000000000010;
	mem_main[1]=32'b00000000000000000000000000000100;
end
endmodule

/* ALU module 
module alu(ALUctl, A, B, ALUOut,enable,zerobit);
input [2:0] ALUctl;
input [31:0] A,B;
output reg [31:0] ALUOut;
output reg enable ;
output reg zerobit;
always @(ALUctl, A, B) begin 
     enable =0;
	case (ALUctl)
        0: begin ALUOut <= A & B; enable <=1; end
        1: begin ALUOut <= A | B; enable <=1; end
        2: begin ALUOut <= A + B; enable <=1; end
        3: begin ALUOut <= A - B; enable <=1; end
		4: begin ALUOut <= A ^ B; enable <=1; end
        5: begin ALUOut <= A < B ? 1 : 0; enable <=1; end
		6: begin ALUOut <= $signed(A) < $signed(B) ? 1 : 0;enable <=1; end
        default: begin ALUOut <= 0;enable <=0; end
    endcase
	if(ALUOut==0) assign zerobit=1;
	else assign zerobit=0;
end
endmodule
*/

module alu(ALUctl, A, B, ALUOut,enable, zerobit);
input [2:0] ALUctl;
input [31:0] A,B;
output reg [31:0] ALUOut;
output reg enable ;
output reg zerobit;
always @(ALUctl, A, B) begin 
     enable =0;
	case (ALUctl)
        0: begin ALUOut <= A & B; enable <=1; end
        1: begin ALUOut <= A | B; enable <=1; end
        2: begin ALUOut <= A + B; enable <=1; end
        3: begin ALUOut <= A - B; enable <=1; end
		4: begin ALUOut <= A ^ B; enable <=1; end
        5: begin ALUOut <= A < B ? 1 : 0; enable <=1; end
		6: begin if(A[31]^B[31]) begin ALUOut <= A > B ? 1 : 0;enable <=1; end else begin ALUOut <= A > B ? 1 : 0;enable <=1; end end 
        default: begin ALUOut <= 0;enable <=0; end
    endcase
	if(ALUOut==0) assign zerobit=1;
	else assign zerobit=0;
end
endmodule


//controller module 
module ctrl (input[5:0] func, opcode, input enable, output reg [2:0] ALUctl, output I_type, write_enable, mem_read, mem_write, memToRegister, bne_checker, beq_checker, jr_checker, jt_checker);
wire load_checker, store_checker, jr, jt;
or (I_type, opcode[0], opcode[1], opcode[2], opcode[3], opcode[4], opcode[5]);
and(load_checker, opcode[5], ~opcode[4], ~opcode[3], ~opcode[2], opcode[1], opcode[0]);
and(store_checker, opcode[5], ~opcode[4], opcode[3], ~opcode[2], opcode[1], opcode[0]);
and(beq_checker, ~opcode[5], ~opcode[4], ~opcode[3], opcode[2], ~opcode[1], ~opcode[0]);
and(bne_checker, ~opcode[5], ~opcode[4], ~opcode[3], opcode[2], ~opcode[1], opcode[0]);
and(jt, ~opcode[5], ~opcode[4], ~opcode[3], ~opcode[2], opcode[1], ~opcode[0]);
and(jr, ~func[5], ~func[4], func[3], ~func[2], ~func[1], ~func[0]);
assign jr_checker=jr&&~I_type;
assign write_enable=enable&&~(store_checker)&&~(beq_checker)&&~(bne_checker)&&~(jr_checker)&&~(jt_checker);
assign mem_read=load_checker;
assign mem_write=store_checker;
assign memToRegister=mem_read;
assign jt_checker=jt;
always @ (*) begin
if (I_type) begin
	case (opcode)
		6'b000100 : ALUctl=3 ;
		6'b000101 : ALUctl=3 ;
		6'b101011 : ALUctl=2 ;
		6'b100011 : ALUctl=2 ; 
		6'b001000 : ALUctl=2 ;
		6'b001101 : ALUctl=1 ;
		6'b001100 : ALUctl=0 ;
		6'b001010 : ALUctl=6 ;
		default   : ALUctl=7 ;
	endcase
end
else begin
	case (func)
		6'b100000 : ALUctl=2 ;
		6'b100010 : ALUctl=3 ;
		6'b100101 : ALUctl=1 ;
		6'b100110 : ALUctl=4 ;
		6'b100100 : ALUctl=0 ;
		6'b101010 : ALUctl=6 ;
		6'b101011 : ALUctl=5 ;
		6'b101011 : ALUctl=5 ;
		default   : ALUctl=7 ; 
	endcase
	
end
end
endmodule