//Ram Bhattarai
//ECE 544
//Detects pulse width modulation
//Hardware Pulse width detection module
//Used to detect high and low values of the pulse width
//Uses the RGB LEds value for the detection

module pwm_detection(
input clk,
input rst,
input pwm_sig,
output reg [31:0] high_sig,
output reg [31:0] low_sig);


//Temporary signal and counter
reg prev_pwm_sig;
reg [31:0] cntr;


always@(posedge clk)
begin
	//Active low reset
	if(!rst)              
	begin
			high_sig<=32'b0;
			low_sig<=32'b0;
			prev_pwm_sig<=1'b0;
			cntr<=32'b0;
	end
	//Pulse is High
	else if(pwm_sig==1'b1)
	begin
		if(prev_pwm_sig == pwm_sig)      //Pulse signal and previous pulse signal matches, increment the counter
		begin
			cntr<=cntr+1'b1;
		end
			
		else                             //Otherwise set the low signal pulse to the counter value
		begin
			low_sig<=cntr;
			prev_pwm_sig<=1'b1;          //set the previous pulse signal to 1, since we detected this
			cntr<=32'b0;                 //reset the counter
		end
	end
	
	//Pulse is low
	else if(pwm_sig==1'b0)
	begin
		if(prev_pwm_sig==pwm_sig)		//Pulse signal and previous pulse signal matches, increment the counter
		begin
			cntr<=cntr+1'b1;
		end
			
		else                          //Otherwise set the high signal pulse to the counter value
		begin
			high_sig<=cntr;
			prev_pwm_sig<=pwm_sig;   //set the previous pulse signal to 0, since we detected this
			cntr<=32'b0;              //reset the counter
		end
	end
end
endmodule



