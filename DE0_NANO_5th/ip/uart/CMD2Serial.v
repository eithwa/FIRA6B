// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert feedback to serial
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/08/05 :|  Initial Version
// --------------------------------------------------------------------

`default_nettype  none
module CMD2Serial (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 	// 50MHz
input				iRst_n,	// Reset
input				iSend_Ready,
input				iTx_busy,
input		[14:0]	iFB_Motor1,		// Feedback of motor1
input		[14:0]	iFB_Motor2,		// Feedback of motor2
input		[14:0]	iFB_Motor3,		// Feedback of motor3
input		[14:0]	iFB_Motor4,		// Feedback of motor4
input				iDIR_Motor1,		// Direction of motor1
input				iDIR_Motor2,		// Direction of motor2
input				iDIR_Motor3,		// Direction of motor3
input				iDIR_Motor4,		// Direction of motor4
output	reg			oTx_send,
output	reg	[7:0]	oTx_data
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	3;
parameter SEND	=	3'b001;
parameter WAIT	=	3'b010;
parameter END	=	3'b100;

parameter PACKAGE_SIZE	=	10;
parameter STREAM_SIZE	=	PACKAGE_SIZE * 8;
parameter DELAY			=	1000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================

reg	[STREAM_SIZE-1:0]	rData;

reg	[SIZE-1:0]	state;

reg				rSend_Ready;

reg				rSend;

reg		[7:0]	rCNT_Package;

reg		[15:0]	rCNT;

reg				rTx_busy;

//=============================================================================
// Structural coding
//=============================================================================

always @(posedge iCLK) begin
	if ( (~rSend_Ready & iSend_Ready) & ~rSend) begin
		rSend	<=	1;
	end
	rSend_Ready	<=	iSend_Ready;

	if (!iRst_n) begin	// Reset
		state	<=	SEND;
		rSend	<=	0;
		rCNT	<=	0;
		rCNT_Package	<=	0;
		rData[7:0]		<=	8'hFF;
		rData[15:8]		<=	8'hFA;
		rData[23:16]	<=	0;
		rData[31:24]	<=	0;
		rData[39:32]	<=	0;
		rData[47:40]	<=	0;
		rData[55:48]	<=	0;
		rData[63:56]	<=	0;
		rData[71:64]	<=	0;
		rData[79:72]	<=	0;
		
	end
	// Combine Data
	else if (rSend) begin
		case(state)
			SEND:
				begin
					oTx_data	<=	rData[7:0];
					oTx_send	<=	1'b1;
					if (rTx_busy & ~iTx_busy) begin	// Delay Signal
						oTx_send	<=	1'b0;
						rData		<=	{rData[7:0], rData[STREAM_SIZE-1:8]};
						state		<=	WAIT;
					end
				end
			WAIT:
				begin
					if (rCNT < DELAY) begin
						rCNT	<=	rCNT + 1;	// up count
					end
					else begin
						rCNT	<=	0;
						state	<=	END;
					end
				end
			END:
				begin
					if (rCNT_Package < PACKAGE_SIZE-1) begin
						rCNT_Package	<=	rCNT_Package + 1;	// up count
						state			<=	SEND;
					end
					else begin
						rCNT_Package	<=	0;
						state			<=	SEND;
						rSend			<=	0;
					end
				end
		endcase
	end
	else begin
		rData[7:0]		<=	8'hFF;
		rData[15:8]		<=	8'hFA;
		rData[23:16]	<=	{iDIR_Motor1, iFB_Motor1[14:8]};
		rData[31:24]	<=	iFB_Motor1[7:0];
		rData[39:32]	<=	{iDIR_Motor2, iFB_Motor2[14:8]};
		rData[47:40]	<=	iFB_Motor2[7:0];
		rData[55:48]	<=	{iDIR_Motor3, iFB_Motor3[14:8]};
		rData[63:56]	<=	iFB_Motor3[7:0];
		rData[71:64]	<=	{iDIR_Motor4, iFB_Motor4[14:8]};
		rData[79:72]	<=	iFB_Motor4[7:0];
	end
	rTx_busy	<=	iTx_busy;
end

endmodule