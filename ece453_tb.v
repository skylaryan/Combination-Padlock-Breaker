module ece453_tb();

reg clk, reset,s,t;
wire [31:0] y, u;
wire z;
reg [3:0] w;
reg [4:0] r;
reg [31:0] v, x;
reg sigA, sigB;
wire servoSig, cw, ccw;
wire correctPos;
wire blocked;
wire adcComplete;
reg adcInput;
wire adcConv;
wire adcClock;
wire adcVal;
wire setEncVal;
/*ece453 ece453(  
  .clk(clk),
  .reset(reset),
  .slave_address(r),
  .slave_read(s),
  .slave_write(t),
  .slave_readdata(u),
  .slave_writedata(v),
  .slave_byteenable(w),
  .gpio_inputs(x),
  .gpio_outputs(y),
  .irq_out(z)
);*/
wire [8:0] dig1, dig2, dig3;
reg [8:0] location;
wire [4:0] current_state;
wire [1:0] servoPos;
reg pause, resume, toSeven;
testBit testBit(
	 .dig1(dig1),
	 .dig2(dig2),
	 .dig3(dig3)
);
	adc adc
	(
		.clk(clk),
		.reset(reset),
		.adcVal(adcVal),
		.analogSig(adcInput),
		.adcConv(adcConv),
		.adcClock(adcClock),
		.adcComplete(adcComplete)
	);

/*  servoMotor servoMotor
	(
		.clk(clk),
		.reset(reset),
		.servoPos(servoPos),
		.servoSig(servoSig),
		.adcVal(adcVal),
		.correctPos(correctPos),
		.blocked(blocked)
	);*/
/*opticalEnc opticalEnc
	(
		.clk(clk),
		.reset(reset),
		.sigA(sigA),
		.sigB(sigB),
		.location(location),
		.dig1(dig1),
	 	.dig2(dig2),
	 	.dig3(dig3)
	);*/
	wire [6:0] HEX2, HEX1;
wire [3:0] ones, tens;
dec_to_7_seg combo_logic
(.x(location),
.z({HEX2, HEX1}),
.ones(ones),
.tens(tens)
);
wire ms1, ms2, step, stpEn, stpRst, stpdir;

reg direction;
reg solved;
reg [5:0] dialLoc;
	/*stepperMotor stepperMotor
	(
		.clk(clk),
		.reset(reset),
		.direction(direction),
		.solved(solved),
		.dialLoc(dialLoc),
		.stpDir(stpDir),
		.stpRst(stpRst),
		.stpEn(stpEn),
		.step(step),
		.ms1(ms1),
		.ms2(ms2)
	);*/
	lockThief lockThief
	(
		.clk(clk),
		.reset(reset),
		.opEncLoc(location),
		.step(step),
		.dir(stpDir),
		.slp(stpSlp),
		.en(stpEn),
		.ms1(ms1),
		.ms2(ms2),
		.stepRst(stpRst),
		.servoPos(servoPos),
		.current_state(current_state),
		.setEncVal(setEncVal),
		.pause(pause),
.resume(resume),
.toSeven(toSeven)
	);
	
initial begin
clk = 0;
direction = 0;
location = 9'b0;
solved = 0;
sigA = 0;
sigB = 1;
dialLoc = 6'b0;
pause = 1'b0;
resume = 1'b0;
toSeven = 1'b0;
reset = 0;
w = 4'b0;
r = 5'b0;
v = 32'b0;
x = 32'b0;
adcInput = 1'b1;
#1 reset = 1;
#0 reset = 0;

/*#100;
adcVal = 12'hFFF;
#100;
servoPos = 2'b10;
#100;
adcVal = 12'h0;
#100;
servoPos = 2'b01;
#100;
adcVal = 12'hfff;
#100;
adcVal = 12'h3b0;*/
end

always begin
	#1 clk = ~clk;
end
always@(negedge setEncVal) begin
	#30 location = location - 9'b0011;
end
always begin
	#3 sigA = ~sigA;
end
always@(posedge adcComplete) begin
//$stop;
end

always@(posedge clk) begin
	location<= location  + 1;
if(location == 9'd300) begin
location <= 9'b0;
end
end
endmodule
///afdkjsadfjsadf;lkajds;lkajdsf;lkasjf;lksajdf;lkjsdflksajf
	

module dec_to_7_seg(
	input [8:0]x,
	output [13:0]z,
	output [3:0] ones,
	output [3:0] tens
);
wire [8:0] inter, oneDig;
assign inter = (x > 9'b011011101) ? 9'b011011101:
						(x > 9'b010010010) ? 9'b010010010:
						(x > 9'b01000111) ? 9'b01000111:
						9'b0;
						
assign z[13:7] = (x > 9'b011011101) ? 7'b0110000: //3
						(x > 9'b010010010) ? 7'b0100100: //2
						(x > 9'b01000111) ? 7'b1111001: //1
						7'b1000000; //0
assign oneDig = x - inter;

assign z[6:0]  = (oneDig > 9'b0111111) ? 7'b0010000: //9
(oneDig > 9'b0111000) ? 7'b0000000://8
(oneDig > 9'b0110000) ? 7'b1111000://7
(oneDig > 9'b0101001) ? 7'b0000010://6
(oneDig > 9'b0100001) ? 7'b0010010://5
(oneDig > 9'b011010) ? 7'b0011001://4
(oneDig > 9'b010010) ? 7'b0110000://3
(oneDig > 9'b01011) ? 7'b0100100://2
(oneDig > 9'b011) ? 7'b1111001://1
7'b1000000; //0

assign tens = (x > 9'b011011101) ? 4'd3: //3
						(x > 9'b010010010) ? 4'd2: //2
						(x > 9'b01000111) ? 4'd1: //1
						4'd0; //0
assign ones  = (oneDig > 9'b0111111) ? 4'd9: //9
(oneDig > 9'b0111000) ? 4'd8://8
(oneDig > 9'b0110000) ? 4'd7://7
(oneDig > 9'b0101001) ? 4'd6://6
(oneDig > 9'b0100001) ? 4'd5://5
(oneDig > 9'b011010) ? 4'd4://4
(oneDig > 9'b010010) ? 4'd3://3
(oneDig > 9'b01011) ? 4'd2://2
(oneDig > 9'b011) ? 4'd1://1
4'd0; //0
endmodule