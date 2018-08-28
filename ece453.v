
/*

  Author:  Joe Krachey
  Date:  01/10/2017

*/


module ece453(
  // signals to connect to an Avalon clock source interface
  clk,
  reset,
  // signals to connect to an Avalon-MM slave interface
  slave_address,
  slave_read,
  slave_write,
  slave_readdata,
  slave_writedata,
  slave_byteenable,
  gpio_inputs,
  gpio_outputs,
  irq_out
);

  //*******************************************************************
  // Module Interface
  //*******************************************************************
  input clk;
  input reset;
  // slave interface
  input [4:0] slave_address;
  input slave_read;
  input slave_write;
  output wire [31:0] slave_readdata;
  input [31:0] slave_writedata;
  input [3:0] slave_byteenable;

  input [31:0] gpio_inputs;
  output [31:0] gpio_outputs;
  output wire irq_out;

  `include "ece453.vh"

  
  //*******************************************************************
  // Register Set
  //*******************************************************************
  reg  [31:0] dev_id_r;
  reg  [31:0] control_r;
  reg  [31:0] status_r; 
  reg  [31:0] im_r;
  reg  [31:0] irq_r;
  reg  [31:0] gpio_in_r;
  reg  [31:0] gpio_out_r;

   //*******************************************************************
  // Wires/Reg
  //*******************************************************************
  wire  [31:0] control_in;
  wire  [31:0] status_in;
  wire  [31:0] im_in;
  reg   [31:0] irq_in;
  wire  [31:0] gpio_in;
  wire  [31:0] gpio_out;
  reg   [31:0] gpio_in_irqs;
  wire  [2:0]  fsm_state;
  wire  [3:0]  fsm_leds;
  wire         debounced_key;
  
  //*******************************************************************
  // LED display wires/reg
  //*******************************************************************
	wire [3:0] dig0, dig1, dig2, dig3, dig4, dig5;
	wire [6:0] ledSeg;
	wire dig0En, dig1En, dig2En, dig3En, dig4En, dig5En;
	//test Digits
	assign dig0 = 4'b1000;
   assign dig1 = 4'b0011;
	assign dig2 = 4'b0100;
	assign dig3 = 4'b0000;
	assign dig4 = 4'b0111;
	assign dig5 = 4'b0110;
	//*******************************************************************
  // Stepper Signals
  //*******************************************************************	
	wire step, stpSlp, stpRst, stpEn, ms1, ms2, stpDir;
	wire [4:0] current_stateL;
	wire [5:0] dialLoc;
	wire direction, solved;
	wire [7:0] steps;
	assign solved = 1'b0;
	assign direction = 1'b0;
	assign dialLoc = 6'b011100;
	wire [9:0] encVal;
	wire [5:0] firstTry, secondTry, thirdTry;
	//*******************************************************************
  // Servo Signals
  //*******************************************************************
	wire servoSig;
	wire [1:0] servoPos;
	//servoPos 00 = farleft, 01 = middle, 10=right
	wire blocked;
	wire correctPos;
	wire [11:0] adcVal;
	//assign adcVal = 12'h000; //highest voltage converted to hex
	wire pause, resume, toSeven;
	//*******************************************************************
  // ADC signals
  //*******************************************************************
  wire adcClock;
  wire adcConv;
  wire adcComplete;
  reg start;
	//*******************************************************************
  // Optical enc oder signals
  //*******************************************************************
  wire [9:0] location;
  wire [9:0] opDig1, opDig2, opDig3;
  //*******************************************************************
  // Register Read Assignments
  //*******************************************************************
  assign slave_readdata = 
        ( (slave_address == DEV_ID_ADDR )    && slave_read )  ? dev_id_r :
        ( (slave_address == CONTROL_ADDR )   && slave_read )  ? control_r:
        ( (slave_address == STATUS_ADDR )    && slave_read )  ? status_r:
        ( (slave_address == IM_ADDR )        && slave_read )  ? im_r :
        ( (slave_address == IRQ_ADDR )       && slave_read )  ? irq_r :
        ( (slave_address == GPIO_IN_ADDR )   && slave_read )  ? gpio_in_r :
        ( (slave_address == GPIO_OUT_ADDR )  && slave_read )  ? gpio_out_r : 32'h00000000 ;

  //*******************************************************************
  // Output Assignments
  //*******************************************************************
   
  // IRQ indicating that an interrupt is active 
  assign irq_out = | (im_r & irq_r);
  assign gpio_outputs = {gpio_out_r[31:0]};

  // Combinational Logic Interrupt.  The interrupt will be generated when the FSM
  // enters state 1 or state 4.
	always @ (*)
	begin
		// Set the default value of the irq registe
		irq_in = irq_r;
		
		if( ((fsm_state == 3'd1) || ( fsm_state == 3'd4)) && ( fsm_state != status_r[2:0]))
		begin
			irq_in = irq_r | 32'h1;
		end
		
		else
		begin
			// Check to see if the IRQ is being cleared.
			if(slave_address == IRQ_ADDR)
			begin
				if( slave_write )
				begin
					irq_in = irq_r & (~slave_writedata);
				end
			end
		end

	end
	
  // Input signals for registers
  assign control_in     = ( (slave_address == CONTROL_ADDR )  && slave_write ) ? (slave_writedata & (CONTROL_FSM_DIR_MASK | CONTROL_FSM_ENABLE_MASK)) : control_r ;
  assign status_in      = {29'h0, fsm_state };
  assign im_in          = ( (slave_address == IM_ADDR )   && slave_write ) ? slave_writedata : im_r;
  assign gpio_in        = gpio_inputs;
  assign resetI = debounced_key || reset;
  //assign gpio_out       = {fsm_state, testLEDBit,23'h7FFFE8, fsm_leds}; //servoSig

  /*assign gpio_out       = { servoSig, dig2En,adcClock,dig3En,1'b0,dig0En,adcConv,dig1En, 
									3'b0,ledSeg[6],1'b0,ledSeg[5],1'b0,ledSeg[4],
									1'b0,ledSeg[3],stpDir,ledSeg[2],ms2,ledSeg[1],ms1,ledSeg[0],
									1'b0,dig4En,1'b0,dig5En,step,stpRst,stpEn,stpSlp};*/
assign gpio_out       = { servoSig, location, current_stateL,
									2'b0, stpDir,ledSeg[2],ms2,ledSeg[1],ms1,ledSeg[0],
									1'b0,dig4En,1'b0,dig5En,step,stpRst,stpEn,stpSlp};
/*assign gpio_out       = { servoSig, firstTry, secondTry, thirdTry[5:2],
									 stpDir,thirdTry[1],ms2,thirdTry[0],ms1,ledSeg[0],
									1'b0,dig4En,1'b0,dig5En,step,stpRst,stpEn,stpSlp};*/

  //*******************************************************************
  // Registers
  //*******************************************************************
  always @ (posedge clk or posedge reset)
  begin
    if (reset == 1)
    begin
      dev_id_r    <= 32'hECE45318;
      control_r   <= 32'h00000000;
      status_r    <= 32'h00000000;
      im_r        <= 32'h00000000;
      irq_r       <= 32'h00000000;
      gpio_in_r   <= 32'h00000000;
      gpio_out_r  <= 32'h00000000;
    end
    
    else
    begin
      dev_id_r    <= 32'hECE45318;
      control_r   <= control_in;
      status_r    <= status_in;
      im_r        <= im_in;
      irq_r       <= irq_in;
      gpio_in_r   <= gpio_in;
      gpio_out_r  <= gpio_out;
    end
  end

  // Debounce the button that controls the state machine 
  ece453_debounce push_button
  (
	  .clk(clk),
	  .reset(resetI),
    .button_in(gpio_inputs[0]),
    .button_out(debounced_key)
  ); 

    ece453_debounce push_button1
  (
	  .clk(clk),
	  .reset(resetI),
    .button_in(gpio_inputs[1]),
    .button_out(pause)
  );
  
    ece453_debounce push_button2
  (
	  .clk(clk),
	  .reset(resetI),
    .button_in(gpio_inputs[2]),
    .button_out(resume)
  );
  
      ece453_debounce push_button3
  (
	  .clk(clk),
	  .reset(resetI),
    .button_in(gpio_inputs[3]),
    .button_out(toSeven)
  );

  // Determine if the LED should be moved.
  ece453_fsm_example ece453_fsm
  (
	  .clk(clk),
	  .reset(resetI),
    .fsm_enable(control_r[CONTROL_FSM_ENABLE_BIT_NUM]),
    .button(debounced_key),
    .direction(control_r[CONTROL_FSM_DIR_BIT_NUM]),
    .led_out(fsm_leds), 
    .current_state(fsm_state)
  );
  
  comboDigi comboDigi
  (
	 .clk(clk),
	 .reset(resetI),
	 .dig0(dig0),
     .dig1(dig1),
    .dig2(dig2),
    .dig3(dig3),
	 .dig4(dig4),
    .dig5(dig5), 
	 .outSegments(ledSeg),
	 .dig0En(dig0En),
    .dig1En(dig1En),
	 .dig2En(dig2En),
	 .dig3En(dig3En),
	 .dig4En(dig4En),
	 .dig5En(dig5En)
  );
  //.feedback(gpio_inputs[31]), input to spi interface from ADC
  servoMotor servoMotor
	(
		.clk(clk),
		.reset(resetI),
		.servoPos(servoPos),
		.servoSig(servoSig),
		.adcVal(adcVal),
		.correctPos(correctPos),
		.blocked(blocked)
	);
	
opticalEnc opticalEnc
	(
		.clk(clk),
		.reset(resetI),
		.sigA(gpio_inputs[30]),
		.sigB(gpio_inputs[29]),
		.location(location)
	);
	
	adc adc
	(
		.clk(clk),
		.reset(resetI),
		.adcVal(adcVal),
		.analogSig(gpio_inputs[31]),
		.adcConv(adcConv),
		.adcClock(adcClock),
		.adcComplete(adcComplete),
		.start(start)
	);
		
	/*stepperMotor stepperMotor
	(
		.clk(clk),
		.reset(resetI),
		.direction(direction),
		.solved(solved),
		.dialLoc(dialLoc),
		.stpDir(stpDir),
		.stpRst(stpRst),
		.stpEn(stpEn),
		.step(step),
		.ms1(ms1),
		.ms2(ms2),
		.steps(steps),
		.stpSlp(stpSlp)
	);
	*/

	lockThief lockThief
	(
		.clk(clk),
		.reset(resetI),
		.opEncLoc(location),
		.step(step),
		.dir(stpDir),
		.slp(stpSlp),
		.en(stpEn),
		.ms1(ms1),
		.ms2(ms2),
		.stepRst(stpRst),
		.servoPos(servoPos),
		.current_state(current_stateL),
		.encVal(encVal),
		.pause(pause),
		.resume(resume),
		.toSeven(toSeven),
		.firstTry(firstTry),
		.secondTry(secondTry),
		.thirdTry(thirdTry)
	);
	
endmodule

module lockThief(
	input clk,
	input reset,
	input [9:0] opEncLoc,
	output reg step,
	output reg dir,
	output reg slp,
	output reg en,
	output reg ms1,
	output reg ms2,
	output reg stepRst,
	output reg [1:0] servoPos,
	output reg [4:0] current_state,
	output reg [9:0] encVal,
	output reg setEncVal,
	input pause,
	input resume,
	input toSeven,
	output reg [5:0] firstTry,
	output reg [5:0] secondTry,
	output reg [5:0] thirdTry
);
//reg[3:0] current_state;
reg[4:0] next_state;
reg [27:0] counter;
//reg [8:0] encVal;
reg rstCnt;
//reg setEncVal;
wire [27:0] max, half;
assign max = 28'b01111010000100100000;//26'b010011000100101101000000;
assign half = 28'b0111101000010010000;//26'b01001100010010110100000;
//assign max = 28'b010;//26'b010011000100101101000000;
//assign half = 28'b01;//26'b01001100010010110100000;
reg [3:0] resCnt;
reg [8:0] res1R, res2R, res3R, res4R, res5R, res6R, res7R, res8R, res9R, res10R, res11R, res12R;
reg [8:0] res1L, res2L, res3L, res4L, res5L, res6L, res7L, res8L, res9L, res10L, res11L, res12L;
reg incRes, rstRes;
reg [8:0] dial1, dial2, dial3;
reg setTries;
wire [27:0] qtSec;
//assign qtSec = 28'b010;
assign qtSec = 28'b0101111101011110000100000;
wire [27:0] halfSec;
//assign halfSec = 28'b01100;
assign halfSec = 28'b01011111010111100001000000;
wire [27:0] oneSec;
assign oneSec = 28'b010111110101111000010000000;
//assign oneSec= 28'b0110;
wire [27:0] threeSec;
assign threeSec = 28'b1000111100001101000110000000;
//assign threeSec = 28'b01100;
reg [9:0] rotations;
reg incRotations, rstRotations;
reg [9:0] thresh;
reg setThresh;
wire [9:0] threshr, currThresh;
reg [3:0] stpCnt;
reg rstStpCnt, incStpCnt;
reg [9:0] currRes;
reg setCurrRes;
reg holdup;
reg [3:0] firstNum, secondNum;
reg incFirst, incSecond, rstFirst, rstSecond; 
reg rotLock, setRotLock, rstRotLock;
assign threshr = (opEncLoc > encVal) ? (opEncLoc - encVal):
					(encVal - opEncLoc);
assign currThresh = opEncLoc - currRes;
reg [9:0] halfResPt;
reg setHalf;
reg [9:0] hardCodeStp;
reg rstHCStp, incHCStp;
//each op enc location is 7.5 values on the dial. each 4 values on the dial is 30 enc locations
wire [9:0] firstThresh, secondThresh, thirdThresh;
assign firstThresh = (firstNum == 4'b0000) ? 10'b011110111:
						(firstNum == 4'b0001) ? 10'b0110100:
						(firstNum == 4'b0010) ? 10'b01010010:
						(firstNum == 4'b0011) ? 10'b01110000:
						(firstNum == 4'b0100) ? 10'b010001110:
						(firstNum == 4'b0101) ? 10'b010101100:
						(firstNum == 4'b0110) ? 10'b011001010:
						(firstNum == 4'b0111) ? 10'b011101000:
						(firstNum == 4'b1000) ? 10'b100000110:
						10'b100100100;
assign secondThresh = (secondNum == 4'b0000) ? 10'b01110000:
						(secondNum == 4'b0001) ? 10'b0100101:
						(secondNum == 4'b0010) ? 10'b01000011:
						(secondNum == 4'b0011) ? 10'b01100001:
						(secondNum == 4'b0100) ? 10'b01111111:
						(secondNum == 4'b0101) ? 10'b010011101:
						(secondNum == 4'b0110) ? 10'b010111011:
						(secondNum == 4'b0111) ? 10'b011011001:
						(secondNum == 4'b1000) ? 10'b011110111:
						10'b100010101;
assign thirdThresh = 10'b100010101;
wire [9:0] firstDig, secondDig, thirdDig;
assign firstDig = (firstThresh > opEncLoc) ? (firstThresh - opEncLoc):
					(opEncLoc - firstThresh);
assign secondDig = (secondThresh > opEncLoc) ? (secondThresh - opEncLoc):
					(opEncLoc - secondThresh);	
assign thirdDig = (thirdThresh > opEncLoc) ? (thirdThresh - opEncLoc):
					(opEncLoc - thirdThresh);	

reg [9:0] firstB, secondB, thirdB, bStp;	
reg incB, rstBStp, incBStp;
wire [9:0] spin1, spin2, spin3;
assign spin1 = firstB + 10'b011001000;
assign spin2 = (secondB > firstB) ? (10'b0110010000 + firstB - secondB):
			(firstB + 10'b011001000 - secondB);
assign spin3 = (thirdB > secondB) ? (thirdB - secondB):
		(10'b011001000 + thirdB - secondB);
always@(posedge clk or posedge reset) begin
	if(reset == 1'b1) begin
		current_state <= 5'b0;
		counter <= 28'b0;
		resCnt <= 4'b0;
		rotations <= 10'b0;
		encVal <= 10'b0;
		currRes <= 10'b0;
		stpCnt <= 4'b0;
		thresh <= 10'b0;
		holdup <= 1'b0;
		firstNum <= 4'b0;
		secondNum <= 4'b0;
		rotLock <= 1'b0;
		halfResPt <= 10'b0;
		firstTry <= 6'b0;
		secondTry <=6'b0;
		thirdTry <=6'b0;
		firstB <= 10'b0; 
		secondB <= 10'b0;
		thirdB <= 10'b0;
		bStp <= 10'b0;
		hardCodeStp <= 10'b0;
	end else begin
		if(pause == 1'b1) begin
			holdup <= 1'b1;
		end else if(resume == 1'b1) begin
			holdup <= 1'b0;
		end
		if(rstCnt == 1'b1) begin
			counter <= 28'b0;
		end else begin
			counter <= counter + 1;
		end
		if(rstRes == 1'b1) begin
			resCnt <= 4'b0;
		end else if(incRes == 1'b1) begin
			resCnt <= resCnt + 1;
		end
		if(rstRotations == 1'b1) begin
			rotations <= 10'b0;
		end else if(incRotations == 1'b1) begin
			rotations <= rotations + 1;
		end
		if(setEncVal == 1'b1) begin
				encVal <= opEncLoc;
		end
		if(setThresh == 1'b1) begin
			if(opEncLoc > encVal) begin
				thresh <= opEncLoc - encVal;
			end else begin
				thresh <= encVal - opEncLoc;
			end
		end
		if(rstStpCnt == 1'b1) begin
			stpCnt <= 4'b0;
		end else if(incStpCnt == 1'b1) begin
			stpCnt <= stpCnt + 1;
		end
		if(setCurrRes == 1'b1) begin
			currRes <= opEncLoc;
		end
		if(rstFirst == 1'b1) begin
			firstNum <= 4'b0;	
		end else if(incFirst == 1'b1) begin
			firstNum <= firstNum + 1;
		end
		if(rstSecond == 1'b1) begin
			secondNum <= 4'b0;	
		end else if(incSecond == 1'b1) begin
			secondNum <= secondNum + 1;
		end
		if(rstRotLock == 1'b1) begin
			rotLock <= 1'b0;
		end else if(setRotLock == 1'b1) begin
			rotLock <= ~rotLock;
		end
		if(setHalf == 1'b1) begin
			halfResPt <= opEncLoc;
		end
		if(setTries == 1'b1) begin
			firstTry <= firstThresh >> 4'b0111;
			secondTry <= secondThresh >> 4'b0111;
			thirdTry <= thirdThresh >> 4'b0111;
		end
		if(rstHCStp == 1'b1) begin
			hardCodeStp <= 10'b0;
		end else if(incHCStp == 1'b1) begin
			hardCodeStp <= hardCodeStp + 1;
		end
		if(incB == 1'b1) begin
			if(thirdB > 10'b011000010) begin
				thirdB <= 10'b0;
				if(secondB > 10'b011000010) begin
					secondB <= 10'b0;
					firstB <= firstB + 10'b0101;
				end else begin
					secondB <= secondB + 10'b0101;
				end
			end else begin
				thirdB <= thirdB + 10'b0101;
			end
		end
		if(rstBStp == 1'b1) begin
			bStp <= 10'b0;
		end else if(incBStp == 1'b1) begin
			bStp <= bStp + 10'b01;
		end
		current_state <= next_state;
	end
end

always@(*) begin
	next_state = current_state;
	//servoPos = 2'b00;
	rstCnt = 1'b0;
	servoPos = 2'b0;
	step = 1'b0;
	slp = 1'b1;
	en = 1'b0;
	dir = 1'b0; //clockwise
	incRes = 1'b0;
	ms1 = 1'b0;
	ms2 = 1'b0;
	stepRst = 1'b1;
	incRotations = 1'b0;
	rstRotations = 1'b0;
	setEncVal = 1'b0;
	rstRes = 1'b0;
	setCurrRes = 1'b0;
	setThresh = 1'b0;
	incStpCnt = 1'b0;
	rstStpCnt = 1'b0;
	incFirst = 1'b0;
	incSecond = 1'b0;
	rstFirst = 1'b0;
	rstSecond = 1'b0;
	setRotLock = 1'b0;
	rstRotLock = 1'b0;
	setHalf = 1'b0;
	setTries = 1'b0;
	rstHCStp = 1'b0;
	incHCStp = 1'b0;
	incB = 1'b0;
	rstBStp = 1'b0;
	incBStp = 1'b0;
	if(holdup == 1'b0) begin
	case(current_state)
	//servo down
		5'b00000:begin
			servoPos = 2'b00;
			//slp = 1'b0;
			//en = 1'b1;
			if(counter == oneSec) begin
				rstCnt = 1'b1;
				next_state = 5'b00001;
			end
		end
		//servo up
		5'b00001:begin
			servoPos = 2'b01;
			if(counter == oneSec) begin
				rstCnt = 1'b1;
				next_state = 5'b00010;
			end
		end
		//grab enc val and step stepper
		5'b00010:begin
			servoPos = 2'b01;
			if(counter == 28'b0 && stpCnt == 4'b0) begin
				setEncVal = 1'b1;
			end
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				if(stpCnt == 4'b0100) begin
					setThresh = 1'b1;
					incStpCnt = 1'b1;
				end else if(stpCnt == 4'b1010) begin
					rstStpCnt = 1'b1;
					next_state = 5'b00011;
				end else begin
					incStpCnt = 1'b1;
				end
			end
		end
		//compare enc values to see if we found resistance
		5'b00011: begin	
			servoPos = 2'b01;
			/*
			if(opEncLoc == 10'b100101011) begin
				//next_state = 5'b00111;
				next_state = 5'b10001;
				rstCnt = 1'b1;
				//each stepper motor step is equal to 1.5 optical encoder steps
			end else
			*/
			 if(threshr > 10'b01010) begin
				next_state = 5'b00010;
				rstCnt = 1'b1;
				//resistance point found
			end else begin
				if(resCnt == 4'b0000) begin
					res1R = encVal;
				end else if(resCnt == 4'b0001) begin
					res2R = encVal;
				end else if(resCnt == 4'b0010) begin
					res3R = encVal;
				end else if(resCnt == 4'b0011) begin
					res4R = encVal;
				end else if(resCnt == 4'b0100) begin
					res5R = encVal;
				end else if(resCnt == 4'b0101) begin
					res6R = encVal;
				end else if(resCnt == 4'b0110) begin
					res7R = encVal;
				end else if(resCnt == 4'b0111) begin
					res8R = encVal;
				end else if(resCnt == 4'b1000) begin
					res9R = encVal;
				end else if(resCnt == 4'b1001) begin
					res10R = encVal;
				end else if(resCnt == 4'b1010) begin
					res11R = encVal;
				end else if(resCnt == 4'b1011) begin
					res12R = encVal;
				end
				setCurrRes = 1'b1;
				//incRes = 1'b1;
				//maybe do some wiggle testing
				next_state = 5'b01111;
				rstCnt = 1'b1;
			end
		end
		5'b01111: begin
			servoPos = 2'b01;
			if(counter == oneSec) begin
					rstCnt = 1'b1;
					next_state = 5'b00110;
			end
		end
		5'b00110: begin
			servoPos = 2'b01;
			dir = 1'b1;
			if(counter == 28'b0 && stpCnt == 4'b0000) begin
				setEncVal = 1'b1;
			end
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				if(stpCnt == 4'b0100) begin
					setThresh = 1'b1;
					incStpCnt = 1'b1;
				end else if(stpCnt == 4'b1000) begin
					rstStpCnt = 1'b1;
					next_state = 5'b01001;
				end else begin
					incStpCnt = 1'b1;
				end
			end
		end
		5'b01001: begin
			servoPos = 2'b01;
			dir = 1'b1;
			setThresh = 1'b1;
			if(threshr <= 9'b0110) begin
				next_state = 5'b01101;
				if(resCnt == 4'b0000) begin
					res1L = encVal;
				end else if(resCnt == 4'b0001) begin
					res2L = encVal;
				end else if(resCnt == 4'b0010) begin
					res3L = encVal;
				end else if(resCnt == 4'b0011) begin
					res4L = encVal;
				end else if(resCnt == 4'b0100) begin
					res5L = encVal;
				end else if(resCnt == 4'b0101) begin
					res6L = encVal;
				end else if(resCnt == 4'b0110) begin
					res7L = encVal;
				end else if(resCnt == 4'b0111) begin
					res8L = encVal;
				end else if(resCnt == 4'b1000) begin
					res9L = encVal;
				end else if(resCnt == 4'b1001) begin
					res10L = encVal;
				end else if(resCnt == 4'b1010) begin
					res11L = encVal;
				end else if(resCnt == 4'b1011) begin
					res12L = encVal;
					next_state = 5'b10001;
				end			
				incRes = 1'b1;
				//maybe do some wiggle testing
				rstCnt = 1'b1;
			end else begin
				next_state = 5'b00110;
				rstCnt = 1'b1;
			end
		end
		5'b01101: begin
			servoPos = 2'b01;
			if(counter == oneSec) begin
					rstCnt = 1'b1;
					next_state = 5'b01110;
			end
		end
		5'b01110: begin
			dir = 1'b0;
			servoPos = 2'b01;
			step = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				if(stpCnt == 4'b1010) begin
					rstStpCnt = 1'b1;
					next_state = 5'b00100;
				end else begin
					incStpCnt = 1'b1;
				end
			end
		end
		//we found a resistance point, lower servo then step stepper then raise servo
		5'b00100: begin
			servoPos = 2'b01;
			if(counter > qtSec) begin
			servoPos = 2'b00;
			if(counter == threeSec) begin
				rstCnt = 1'b1;
				next_state = 5'b00101;
				servoPos = 2'b00;
			end
			end
		end
		//step while servo is lowered
		5'b00101: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				if(stpCnt == 4'b1000) begin
					rstStpCnt = 1'b1;
					next_state = 4'b1000;
				end else begin
					incStpCnt = 1'b1;
				end
			end
		end
		//let the stepper finish stepping before raising the servo
		5'b01000: begin
			servoPos = 2'b00;
			if(counter == oneSec) begin
				rstCnt = 1'b1;
				next_state = 5'b00001;
			end
		end
		//use resistance points to calculate combos
		5'b00111:begin
			servoPos = 2'b00;
			if(counter == oneSec) begin
				rstCnt = 1'b1;
				next_state = 5'b01010;
			end
		end
		5'b01010: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
			end 
			if(firstDig < 10'b01010 && rotLock == 1'b0) begin
				if(rotations == 10'b0010) begin
					next_state = 5'b01011;
					rstRotations = 1'b1;
					rstCnt = 1'b1;
					rstRotLock = 1'b1;
				end
				incRotations = 1'b1;
				setRotLock = 1'b1;
			end else if(firstDig > 10'b11000) begin
				rstRotLock = 1'b1;
			end
		end
		5'b01011: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
			end
			if(secondDig < 10'b01010 && rotLock == 1'b0) begin
				if(rotations == 10'b0001) begin
					if(secondNum == 4'b1001) begin
						rstSecond = 1'b1;
						incFirst = 1'b1;
					end else begin
						incSecond = 1'b1;
					end
					next_state = 5'b01100;
					rstRotations = 1'b1;
					rstRotLock = 1'b1;
					rstCnt = 1'b1;
				end
				incRotations = 1'b1;
				setRotLock = 1'b1;
			end else if(secondDig > 10'b11000) begin
				rstRotLock = 1'b1;
			end
		end
		5'b01100: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
			end
			if(thirdDig < 10'b01010) begin
				next_state = 5'b10000;
				rstRotLock = 1'b1;
				rstRotations = 1'b1;
			end
		end
		5'b10000: begin
			servoPos = 2'b00;
			if(counter > oneSec) begin
				servoPos = 2'b01;
				if(counter == threeSec) begin
					rstCnt = 1'b1;
					setTries = 1'b1;
					next_state = 5'b01010;
				end
			end
		end
		5'b10001: begin
			servoPos = 2'b11;
			if(counter == oneSec) begin
					rstCnt = 1'b1;
					next_state = 5'b10010;
			end
		end
		//half resistance
		5'b10010: begin
			servoPos = 2'b11;
			step = 1'b0;
			dir = 1'b0;
			if(counter == 28'b0 && stpCnt == 4'b0) begin
				setEncVal = 1'b1;
			end
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				if(stpCnt == 4'b0100) begin
					setThresh = 1'b1;
					incStpCnt = 1'b1;
				end else if(stpCnt == 4'b1010) begin
					rstStpCnt = 1'b1;
					next_state = 5'b10011;
				end else begin
					incStpCnt = 1'b1;
				end
			end
			end
			5'b10011: begin
			servoPos = 2'b11;
			if(threshr > 10'b01010) begin
				next_state = 5'b10010;
				rstCnt = 1'b1;
				//resistance point found
			end else begin
				setHalf = 1'b1;
				setCurrRes = 1'b1;	
				next_state = 5'b00111;
				rstCnt = 1'b1;
			end
			end
			//open the darn thing
			5'b10100: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incHCStp = 1'b1;
			end
			if(hardCodeStp == 10'b1001111011) begin
				next_state = 5'b10101;
				rstHCStp = 1'b1;
			end
			end
			5'b10101: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incHCStp = 1'b1;
			end
			if(hardCodeStp == 10'b0100110110) begin
				next_state = 5'b10110;
				rstHCStp = 1'b1;
			end
			end
			5'b10110: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incHCStp = 1'b1;
			end
			if(hardCodeStp == 10'b01011010) begin
				next_state = 5'b11111;
				rstHCStp = 1'b1;
			end
			end
			5'b11111: begin
				servoPos = 2'b10;
			end
			//bruteforcebaby
			5'b10111: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incBStp = 1'b1;
			end
			if(bStp == spin1) begin
				next_state = 5'b11000;
				rstBStp = 1'b1;
			end
			end
			5'b11000: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incBStp = 1'b1;
			end
			if(bStp == spin2) begin
				next_state = 5'b11001;
				rstBStp = 1'b1;
			end
			end
			5'b11001: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b1;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incBStp = 1'b1;
			end
			if(bStp == spin3) begin
				next_state = 5'b11010;
				rstBStp = 1'b1;
			end
			end
			5'b11010: begin
				servoPos = 2'b10;
				if(counter > halfSec) begin
				servoPos = 2'b00;
				if(counter > oneSec) begin
					rstCnt = 1'b1;
					next_state = 5'b11011;
				end
			end
			end
			5'b11011: begin
			servoPos = 2'b00;
			step = 1'b0;
			dir = 1'b0;
			if(counter < half) begin
				step = 1'b0;
			end else if(counter < max) begin
				step = 1'b1;
			end else if(counter == max) begin
				step = 1'b0;
				rstCnt = 1'b1;
				incBStp = 1'b1;
			end
			if(bStp == thirdB) begin
				next_state = 5'b10111;
				rstBStp = 1'b1;
				incB = 1'b1;
			end
			end
		endcase
		end
		if(toSeven == 1'b1) begin
			next_state = 5'b10100;
			//next_state = 5'b00111;
		end
		if(resume == 1'b1) begin
			next_state = 5'b10111;
		end
end





endmodule
//*****************************************************************************
// Stepper driver control
//*****************************************************************************
module stepperMotor(
	input clk,
	input reset,
	input [5:0] dialLoc,
	input direction,
	input solved,
	output stpSlp,
	output stpRst,
	output reg stpEn,
	output reg step,
	output ms1,
	output ms2,
	output reg stpDir,
	output reg [7:0] steps
);
reg [25:0] counter;
reg rstCnt;
reg prevStep;
reg [1:0] current_state;
reg [1:0] next_state;
reg [1:0] rotCnt;
reg incRot;
reg rstRot;

wire [25:0] max, half;
assign max = 26'b01111010000100100000;//26'b010011000100101101000000;
assign half = 26'b0111101000010010000;//26'b01001100010010110100000;
always@(posedge clk or posedge reset) begin
	if(reset == 1'b1) begin
		counter <= 26'b0;
		steps <= 8'b0;
		current_state <= 2'b0;
		rotCnt <= 2'b0;
		prevStep <= 1'b0;
	end else begin
		if(rstCnt == 1'b1) begin
			counter <= 26'b0;
		end else begin
			counter <= counter + 1;
		end
		if(step == 1'b1 && prevStep == 1'b0) begin
			if(stpDir == 1'b0) begin
				steps <= steps + 1;
				if(steps ==  8'b11001000) begin
					steps <= 8'b0;
				end
			end else begin
				steps <= steps - 1;
				if(steps == 8'b11111111) begin
					steps <= 8'b11000111;
				end
			end
		end
		if(rstRot == 1'b1) begin
			rotCnt <= 2'b0;
		end else if(incRot==1'b1) begin
			rotCnt <= rotCnt + 1;
		end
		prevStep <= step;
		current_state <= next_state;
	end
end

wire [7:0] first;
assign first =8'b10010110;
wire [7:0] second;
assign second = 8'b00110010;
wire [7:0] third;
assign third = 8'b011001;
wire [7:0] fourth;
assign fourth = 8'b00000000;

//assign stpEn = 1'b0;
assign stpRst = 1'b1;
assign ms1 = 1'b0;
assign ms2 = 1'b0;
assign stpSlp = 1'b1;
//assign stpDir = 1'b0;

always@(*) begin
	stpDir = 1'b0;
	rstCnt = 1'b0;
	incRot = 1'b0;
	rstRot = 1'b0;
	stpEn = 1'b0;
	next_state = current_state;
	if(counter < half) begin
		step = 1'b0;
	end else begin
		step = 1'b1;
	end
	if(counter == max) begin
		rstCnt = 1'b1;
	end
	case(current_state)
		2'b00:begin
			stpDir = 1'b0;
			if(rotCnt == 2'b10) begin
				if(steps == first && counter == (max - 26'b01)) begin
					rstRot = 1'b1;
					next_state = 2'b01;
				end
			end else if(steps == first && counter == (max - 26'b01)) begin
				incRot = 1'b1;
			end
		end
		2'b01:begin
			stpDir = 1'b1;
			if(rotCnt == 2'b01) begin
				if(steps == second && counter == (max - 26'b01)) begin
					rstRot = 1'b1;
					next_state = 2'b10;
				end
			end else if(steps == second && counter == (max - 26'b01)) begin
				incRot = 1'b1;
			end
		end
		2'b10:begin
			stpDir = 1'b0;
			if(steps == third && counter == (max - 26'b01)) begin
				next_state = 2'b11;	
			end
		end
		2'b11: begin
			stpDir = 1'b0;
			stpEn = 1'b1;
			if(steps == fourth && counter == (max - 26'b01)) begin
				next_state = 2'b00;
			end
		end
		endcase
end

endmodule
//*****************************************************************************
// ADC control
//*****************************************************************************
module adc(
	input clk,
	input reset,
	output reg [11:0] adcVal,
	input analogSig,
	output reg adcConv,
	output reg adcClock,
	output reg adcComplete,
	input start
);

reg [1:0] current_state;
reg [1:0] next_state;

reg incHighClk;
reg setHighClk;
reg loadVal;

reg [11:0] val;
reg [11:0] counter;

wire [11:0] idle;
assign idle = 12'hFFF;

wire [11:0] tdrp;
assign tdrp = 12'h07F;

wire [11:0] clkLow;
assign clkLow = 12'h03F;

wire [11:0] clkHigh;
assign clkHigh = 12'h7F;

reg rstCounter;

reg [3:0] highClkCount;

always@(posedge clk or posedge reset) begin
	if(reset == 1) begin
		counter <= 12'b0;
		current_state <= 2'b0;
		val <= 12'b0;
		adcVal <= 12'b0;
		highClkCount <= 4'b0;
	end else begin
		if(rstCounter == 1'b1) begin
			counter <= 12'b0;
		end else begin
			counter <= counter + 1;
		end
		if(loadVal == 1'b1) begin
			val <= val << 1;	
			val[0] <= analogSig;	
		end
		if(adcComplete == 1'b1) begin
			adcVal <= val;
		end
		if(setHighClk == 1'b1) begin
			highClkCount <= 4'b0;
		end
		if(incHighClk == 1'b1) begin
			highClkCount <= highClkCount + 1;
		end
		current_state <= next_state;
	end
end

always@(*) begin
		rstCounter = 1'b0;
		adcComplete = 1'b0;
		loadVal = 1'b0;
		setHighClk = 1'b0;
		incHighClk = 1'b0;
		case(current_state)
		2'b00:begin
			adcConv = 1'b1;
			adcClock = 1'b0;
			if(counter == idle || start == 1'b1) begin
				next_state = 2'b01;
				rstCounter = 1'b1;
			end else begin
				next_state = 2'b00;
			end
		end
		2'b01:begin
			adcConv = 1'b0;
			adcClock = 1'b0;
			if(counter == tdrp) begin
				next_state = 2'b10;
				setHighClk = 1'b1;
				rstCounter = 1'b1;
			end else begin
				next_state = 2'b01;
			end
		end
		2'b10:begin
			adcConv = 1'b0;
			adcClock = 1'b0;
			next_state = 2'b10;
			if(counter < clkLow) begin
				adcClock = 1'b1;
				if(counter == 12'b0) begin
					incHighClk = 1'b1;
					if(highClkCount > 4'b0010 && highClkCount < 4'b1111) begin
						loadVal = 1'b1;		
					end
				end				
			end else if(counter == clkHigh) begin
				adcClock = 1'b0;	
				rstCounter = 1'b1;
				if(highClkCount == 4'b1111) begin
					next_state = 2'b00;
					adcComplete = 1'b1;
				end
			end		
		end
		endcase
end

endmodule

//*****************************************************************************
// Optical Encoder control
//*****************************************************************************
module opticalEnc(
	input clk,
	input reset,
	input sigA,
	input sigB,
	output reg [9:0] location
);
	

	
	reg prevA;
	reg prevB;
	//cannot have double edge sensitive inputs, Quartus prohibits it.
	always@(posedge reset or posedge clk) begin
	 	if(reset == 1) begin
			location <= 10'b0;
			prevA <= 1'b0;
		end else begin
			if(prevA == 1'b0 && sigA == 1'b1) begin
				if(sigB == 1'b1) begin
					if(location > 10'b1001010110) begin
						location <= location - 10'b1001010110;
					end else if(location == 10'b1001010110) begin
						location <= 10'b0;
					end else begin
						location <= location + 1;
					end
				end 
				if(sigB == 1'b0) begin
					if(location > 10'b1001010110) begin
						location <= location - 10'b1001010110;
					end else if(location == 10'b000) begin
						location <= 10'b1001010110;
					end else begin
						location <= location - 1;
					end
				end
			end else if(prevB == 1'b0 && sigB == 1'b1) begin
				if(sigA == 1'b1) begin
					if(location > 10'b1001010110) begin
						location <= location - 10'b1001010110;
					end else if(location == 10'b000) begin
						location <= 10'b1001010110;
					end else begin
						location <= location - 1;
					end
				end
				if(sigA == 1'b0) begin
					if(location > 10'b1001010110) begin
						location <= location - 10'b1001010110;
					end else if(location == 10'b1001010110) begin
						location <= 10'b0;
					end else begin
						location <= location + 1;
					end
				end
			end
			prevA <= sigA;
			prevB <= sigB;
		end
	end
endmodule

//*****************************************************************************
// Servo motor
//*****************************************************************************

module servoMotor(
	input clk,
	input reset,
	input [1:0] servoPos,
	output servoSig,
	input [11:0] adcVal,
	output reg correctPos,
	output reg blocked
);
	reg current_state;
	reg next_state;
	reg  [16:0] counter;
	reg rstCnt;
	reg [1:0] prevPos;
	wire  [16:0] max; //5000 if left, 62,500 if middle, 125,000 if right , 28,750 half
 	wire [11:0] middle, right, left, half, currLoc, threshold;
	reg [25:0] timer;
	wire [25:0] oneSec;
	assign oneSec = 26'b10111110101111000010000000;
	reg rstTimer;
	//7FFF = 2.49878 V
	//0000 = 0V
	//0.00122 V per bit
	//0.00122 V  = 001
	//Threshold is +- 8B from the max measured value
	//farRight = 2.47 V = 7E7
	//middle = 1.32 V = 439 
	//farLeft = 0.43 V = 160
	assign right = 12'h7E7;
	assign middle = 12'h439;//439
	assign left = 12'h160;
	assign half = 12'h250;
	assign threshold = 12'h08B;
	//servoPos 00 = farleft, 01 = middle, 10=right
	assign currLoc = (servoPos == 2'b00) ? left:
				(servoPos == 2'b01) ? middle:
				(servoPos == 2'b11) ? half:
				right;

	always@(posedge clk or posedge reset) begin
		if (reset == 1) begin
			counter <= 17'b0;
			current_state <= 1'b0;
			timer <= 26'b0;
			prevPos <= 2'b0;
		end else begin
			if(rstCnt == 1'b1) begin
				counter <= 17'b0;
			end else begin
				counter <= counter + 1;
			end
			current_state <= next_state;
			prevPos <= servoPos;
			if(rstTimer == 1'b1) begin
				timer <= 26'b0;
			end else begin
				timer <= timer + 1;
			end
		end
	end
	
	assign max = (servoPos == 2'b00) ? 17'b00001001110001000:
					(servoPos == 2'b01) ? 17'b01111010000100100:
					(servoPos == 2'b10) ? 17'b11110100001001000:
					(servoPos == 2'b11) ? 17'b0111001100111100:
					17'b0;
					
	assign servoSig = current_state;
	
	//assuming a clock frequency of 50 MHz
	always@(*) begin
		rstCnt = 1'b0;
		rstTimer = 1'b0;
		next_state = current_state;
		if(counter == max) begin
			rstCnt = 1'b1;
			if(current_state == 1'b0) begin
				next_state = 1'b1;
			end else if(current_state == 1'b1) begin
				next_state = 1'b0;
			end
		end
		//reset timer if we go to a new position
		if(prevPos != servoPos) begin
			rstTimer = 1'b1;
		end
	end
	
	//will assert blocked if the adcVal does not get in the correct threshold after one second
	always@(*) begin
		correctPos = 1'b0;
		//left
		if(servoPos == 2'b00) begin
			//adcVal tells us we are far left
			if((currLoc + threshold ) > adcVal) begin
				correctPos = 1'b1;
			end 
		//middle
		end else if(servoPos == 2'b01) begin
			//adcVal tells us we are in the middle
			if(((currLoc - threshold ) < adcVal) && ((currLoc + threshold) > adcVal)) begin
				correctPos = 1'b1;
			end
		//right
		end else begin
			//adcVal tells us we are far right
			if((currLoc - threshold ) < adcVal) begin
				correctPos = 1'b1;
			end
		end
		//if the position is not correct and it has been one second assert blocked
		if(timer > oneSec && correctPos == 1'b0) begin
			blocked = 1'b1;
		end else begin
			blocked = 1'b0;
		end
	end
	
endmodule

//*****************************************************************************
// LED combo display
//*****************************************************************************
module comboDigi(
	input clk,
	input reset,
	input [3:0] dig0,
	input [3:0] dig1,
	input [3:0] dig2,
	input [3:0] dig3,
	input [3:0] dig4,
	input [3:0] dig5,
	output [6:0] outSegments,
	output reg dig0En,
	output reg dig1En,
	output reg dig2En,
	output reg dig3En,
	output reg dig4En,
	output reg dig5En
);
   reg [2:0] current_state;
	reg [2:0] next_state;
	reg [3:0] outDig;
	
	reg  [15:0] counter;
	wire  [15:0] max; 

	assign max = 16'hFFFF;

	always@(posedge clk or posedge reset) begin
		if (reset == 1) begin
			current_state <= 3'b0;
			counter <= 16'b0;
		end else begin
			current_state <= next_state;
			counter <= counter + 1;
		end
	end
	
	always@(*) begin
		next_state = 3'b0;
		outDig = dig0;
		dig0En = 0;
		dig1En = 0;
		dig2En = 0;
		dig3En = 0;
		dig4En = 0;
		dig5En = 1;
		outDig = dig5;
		case(current_state)
		3'b000:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b001;
				end
				//outDig = dig1;
				//dig1En = 1;
		end
		3'b001:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b010;
				end
				//outDig = dig2;
				//dig2En = 1;
		end
		3'b010:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b011;
				end
				//outDig = dig3;
				//dig3En = 1;
		end
		3'b011:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b100;
				end
				//outDig = dig4;
				//dig4En = 1;
		end
		3'b100:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b101;
				end
				outDig = dig5;
				dig5En = 1;
		end
		3'b101:begin
				next_state = current_state;
				if(counter == max) begin
					next_state = 3'b000;
				end
				//outDig = dig0;
				//dig0En = 1;
		end
	endcase
	end
	
	//G
	assign outSegments[0] = (outDig != 4'b0000) && (outDig != 4'b0001) && (outDig != 4'b0111);
	//C
	assign outSegments[1] = (outDig != 4'b0010);
	//D
	assign outSegments[2] = (outDig != 4'b0001) && (outDig != 4'b0111);
	//E
	assign outSegments[3] = (outDig == 4'b0000) || (outDig == 4'b0010) || (outDig == 4'b0110) || (outDig == 4'b1000); // (outDig == 4'b0000) || (outDig == 4'b0010) || (outDig == 4'b0110) || (outDig == 4'b1000);
	//F
	assign outSegments[4] = (outDig != 4'b0001) && (outDig != 4'b0010) && (outDig != 4'b0011) && (outDig != 4'b0111);
	//B
	assign outSegments[5] = (outDig != 4'b0101) && (outDig != 4'b0110);
	//A
	assign outSegments[6] = (outDig != 4'b0001) && (outDig != 4'b0100);
	
	
endmodule

module ece453_fsm_example(
  input clk,
  input reset,
  input fsm_enable,
  input button,
  input direction,
  output reg [3:0] led_out,
  output reg [2:0] current_state 
);

 
  // Include the header file with the state definitions
  `include "ece453_fsm_example.vh"
  
  reg [2:0] next_state;



  // Implement the D Flip Flops used for the FSM as a separate always block
  //
  // For sequential logic, you must use NON-Blocking statements!!!! 
  //
  // The sensitivity list should ONLY be the an edge of the clock AND the reset
  // signal.
  always @ ( posedge clk or posedge reset) 
  begin
    if  (reset == 1) 
    begin
      current_state <= START;
    end 
    else 
    begin
      current_state <= next_state;
    end
  end

  // Implment the combinational logic for the FSM and output logic as
  // a combinational block using BLOCKING statements!!!
  //
  // The sensitivity list should be a *
always@(*) begin
	next_state = ERROR;
	led_out = 4'b1111;
	case(current_state)
		START:begin
			if(fsm_enable) begin
				next_state = LED0;
				led_out = 4'b0001;
			end else begin
				next_state = START;
				led_out = 4'b0000;
			end
		end
		LED0:begin
			if(fsm_enable && button && direction) begin
				next_state = LED1;
				led_out = 4'b0010;
			end else begin
				next_state = LED0;
				led_out = 4'b0001;
			end
		end
		LED1:begin
			if(fsm_enable && button && direction) begin
				next_state = LED2;
				led_out = 4'b0100;
			end else if(fsm_enable && button && ~direction) begin
				next_state = LED0;
				led_out = 4'b0001;
			end else begin
				next_state = LED1;
				led_out = 4'b0010;
			end
		end
		LED2:begin
			if(fsm_enable && button && direction) begin
				next_state = LED3;
				led_out = 4'b1000;
			end else if(fsm_enable && button && ~direction) begin
				next_state = LED1;
				led_out = 4'b0010;
			end else begin
				next_state = LED2;
				led_out = 4'b0100;
			end
		end
		LED3:begin
			if(fsm_enable && button && ~direction) begin
				next_state = LED2;
				led_out = 4'b0100;
			end else begin
				next_state = LED3;
				led_out = 4'b1000;
			end
		end
		ERROR:begin
			led_out = 4'b1111;
		end
	endcase
end

endmodule 

//*****************************************************************************
// ECE453 Button Debounce 
//*****************************************************************************
module ece453_debounce(
  input clk,
  input reset,
  input button_in,
  output reg button_out
);

  
  reg [23:0] debounce_timer_r;
  reg [7:0] samples_r;

  reg [23:0] debounce_timer_in;
  reg [7:0] samples_in;

  // Combinational Logic
  always @ (*) 
  begin
      if(debounce_timer_r == 24'd0)
      begin
        debounce_timer_in = 24'd500000;
        samples_in = ((samples_r << 1) | button_in);
      end
      else
      begin
        debounce_timer_in = debounce_timer_r - 24'd1;
        samples_in = samples_r;
      end

      if( samples_r == 8'h80)
      begin
        button_out = 1;
        samples_in = 8'h00;
      end
      else
      begin
        button_out = 0;
      end

  end

  // Sequential Logic
  always @ ( posedge clk or posedge reset) 
  begin
    if(reset) 
    begin
      samples_r         <= 8'hFF;
      debounce_timer_r  <= 24'd500000;
    end

    else 
    begin
      debounce_timer_r  <= debounce_timer_in;
      samples_r         <= samples_in;
    end
  end

endmodule


