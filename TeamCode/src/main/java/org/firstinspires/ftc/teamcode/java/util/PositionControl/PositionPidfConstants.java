package org.firstinspires.ftc.teamcode.java.util.PositionControl;

public final class PositionPidfConstants {
	//TODO get rid of this in libary
		//US
	public static final PositionPidfController USDrive = new PositionPidfController(0, 0, 0, 0);
	//public static final PIDFController USDrive = new PIDFController(0.00118, 0, 0.00125, 0);
	public static final PositionPidfController USStrafe = new PositionPidfController(0, 0, 0, 0);
	//public static final PIDFController USStrafe = new PIDFController(0.00125, 0, 0, 0);
	//public static final PIDFController USTurn = new PIDFController(0, 0, 0, 0);
	public static final PositionPidfController USTurn = new PositionPidfController(0.35, 0, 0.3695, 0);
		//Israel
	public static final PositionPidfController ISDrive = new PositionPidfController(0,0,0,0);
	public static final PositionPidfController ISStrafe = new PositionPidfController(0,0,0,0);
	public static final PositionPidfController ISTurn = new PositionPidfController(0,0,0,0);
}
