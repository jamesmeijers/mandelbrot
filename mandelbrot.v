module main(KEY, CLOCK_50, 
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B );
    input CLOCK_50;
	 input [3:0] KEY;
	 
	output			VGA_CLK;
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	 
	 reg [8:0] X;
	 reg [7:0] Y;
	 reg signed [19:0] Ci; 
	 reg signed [19:0] Cr;
	 reg print, newC, hold;
	 reg [9:0] counter;
	 wire [5:0] colour;
	 
	 functionUnit mainunit(.Ci(Ci), .Cr(Cr), .printing(print), .newC(newC), 
		  	.clock(CLOCK_50), .reset(KEY[0]), .colour(colour));
		 
	wire signed [19:0] REAL, REALSTEP, IMAG, IMAGSTEP;
	assign REAL = 20'b00100000000000000000;
	assign REALSTEP = 20'b00000000001100000000;
	assign IMAG = 20'b00011000000000000000;
	assign IMAGSTEP = 20'b00000000001100000000;
	 
	 always @(posedge CLOCK_50) begin
		if(!KEY[0])begin
			X <= 9'd320;
			Y <= 8'd0;
  			Cr <= REAL;
   		Ci <= IMAG;
			print <= 1'b0;
			newC <= 1'b0;
			counter <= 10'd0;
		
		end
		else if(newC) begin
			if(X == 9'd0)begin
				X <= 9'd320;
				Cr <= REAL;
				if(Y == 8'd240)begin
					Y <= 8'd0;
					Ci <= IMAG;
				end
				else begin
					Y <= Y + 1'b1;
					Ci <= Ci - IMAGSTEP;
				end
				
			end
			else begin
				X <= X - 1'b1;
				Cr <= Cr - REALSTEP;
			end
			newC <= 1'b0;
			hold <= 1'b0;
			print <= 1'b0;
			counter <= 10'd0;
		end
		else if(print)begin
				print <= 1'b0;
				hold <= 1'b1;
		end
		else if(hold)begin
			hold <= 1'b0;
			newC <= 1'b1;
		end
		
		else if(counter == 10'd300)begin
			print <= 1'b1;
		
		end
		
		else begin
			counter <= counter + 1'b1;
		
		end
		
	end
	 
	 
	 
	 
	 
	 vga_adapter VGA(
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(colour),
			.x(X),
			.y(Y),
			.plot(print),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "320x240";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 2;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
	 
	 
	 
	 
	 
endmodule





module functionUnit(Ci, Cr, printing, newC, clock, reset, colour);
    input [19:0] Ci, Cr;
    input printing, newC, clock, reset;
    output [5:0] colour;
    
    wire [19:0] Xr, Xi;
    wire loadc, multiply, add, diverge, test;
	 wire [5:0] counter;
    
    assign colour = diverge? 6'b111111 : 6'b000000; //black if diverges, white if not
    
    control c1(.printing(printing), .newC(newC), .clock(clock), .diverge(diverge), 
               .loadc(loadc), .multiply(multiply), .add(add), .reset(reset), .test(test));
    
    
    datapath d1(.loadc(loadc), .multiply(multiply), .add(add), .Ci(Ci), .Cr(Cr), 
                .Xr(Xr), .Xi(Xi), .clock(clock), .reset(reset), .test(test), .diverge(diverge), .counter(counter));
    
endmodule




module control(printing, newC, clock, diverge, loadc, multiply, add, reset, test);
    input printing, newC, clock;
    input diverge, reset;
    output reg loadc, multiply, add, test;
    
    reg [2:0] currentstate, nextstate;
    
    localparam 
        START = 3'd0,
        MULT  = 3'd1,
        ADD   = 3'd2,
        TEST  = 3'd3,
        TEST2 = 3'd4,
        HOLD  = 3'd5;
    
    always@(*)begin
        case(currentstate)
            START: nextstate = MULT;
            MULT: nextstate = ADD;
            ADD: nextstate = TEST;
            TEST: nextstate = TEST2;
            TEST2: nextstate = (printing | diverge)? HOLD : MULT;
            HOLD: nextstate = newC? START: HOLD;
            default: nextstate = START;
        endcase
    end
    
    always@(*)begin
        loadc = 1'b0;
        multiply = 1'b0;
        add = 1'b0;
        test = 1'b0;
        case(currentstate)
            START: loadc = 1'b1;
            MULT: multiply = 1'b1;
            ADD: add = 1'b1;
            TEST: test = 1'b1;
        endcase
    end
    
    always@(posedge clock)begin
        if(!reset)
            currentstate <= START;
        else 
            currentstate <= nextstate;
    end
    
    
endmodule


module datapath(loadc, multiply, add, Ci, Cr, Xr, Xi, clock, reset, test, diverge, counter);
    input loadc, multiply, add, clock, reset, test;
    input signed [19:0] Ci, Cr;
    reg signed [19:0] ci, cr;
    output reg signed [19:0] Xr, Xi;
    output reg diverge;
	 output reg [5:0] counter;
    wire signed [19:0] XrXr, XiXi, XrXi; //values of shifted multiplication
	 wire signed [20:0] absval;
	 assign absval = XrXr + XiXi;
    
    mult xrxr(.x(Xr), .y(Xr), .result(XrXr));
    mult xixi(.x(Xi), .y(Xi), .result(XiXi));
    mult xrxy(.x(Xr), .y(Xi), .result(XrXi));//perform the multiplication
    
    always@(posedge clock)begin
        if(reset == 1'b0)begin
            cr <= Cr;
            ci <= Ci;
            Xr <= 20'd0;
            Xi <= 20'd0;
				counter <= 6'd0;
        end
        if(loadc == 1'b1)begin
            cr <= Cr;
            ci <= Ci;
            Xr <= 20'd0;
            Xi <= 20'd0;
        end
        if(multiply == 1'b1)begin
            Xr <= XrXr - XiXi;
            Xi <= 2 * XrXi;
        end
        if(test == 1'b1)begin
            //diverge = Xr[19]^Xr[18] | Xr[19]^Xr[17] | Xi[19]^Xi[18] | Xi[19]^Xi[17];
				diverge = absval[20]^absval[19] | absval[20]^absval[18];
        end
        if(add == 1'b1)begin
				counter <= counter + 1'b1;
            Xr <= Xr + cr;
            Xi <= Xi + ci; //end of an iteration, either stop if diverging, or repeat
        end
    end
    
    
endmodule

module mult(input signed [19:0] x, input signed [19:0] y, output signed [19:0] result);
    reg signed [39:0] multiply;
    always@(*)
        multiply = x * y;
    assign result[19] = x[19]^y[19]; //sign bit
    assign result[18:0] = multiply[34:16]; //top few bits will overflow, bottom few fall of the end
    //but the number shouldn't iterate past 4 regardless, so the overflow doesn't matter
endmodule