package org.firstinspires.ftc.teamcode.java.op_modes.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.movement.AutoDriving;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfConstants;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

//import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
//import org.firstinspires.ftc.teamcode.java.fieldmapping.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.util.*;

@Autonomous(name="theC00lerBasicRemoteAuton", group="auton")
public class ComplexRemoteAuton extends LinearOpMode {

	private final ElapsedTime runtime = new ElapsedTime();
	// Declare OpMode members.
	RobotHardware robot = new RobotHardware();

	private DcMotor frontRightMotor;
	private DcMotor frontLeftMotor;
	private DcMotor backLeftMotor;
	private DcMotor backRightMotor;
	private DcMotor intakeAndDelivery;
	private BNO055IMU imu;
	private AutoDriving autoDriving;
	private PositionPidfController PIDFDrive;
	private PositionPidfController PIDFStrafe;
	private PositionPidfController PIDFTurn;
	public RevColorSensorV3 colorSensor;
	private Servo lowerWobble;
	int movement = 0;
	int red = 0;

	private boolean location;

	//private ElapsedTime runtime = new ElapsedTime();
	@Override
	public void runOpMode() {
		location = true;
		robot.init(hardwareMap);

		imu = robot.imu;

		frontLeftMotor = robot.frontLeftMotor;
		frontRightMotor = robot.frontRightMotor;
		backRightMotor = robot.backRightMotor;
		backLeftMotor = robot.backLeftMotor;
		intakeAndDelivery = robot.intakeAndDelivery;
		colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
		//lowerWobble = robot.lowerWobble;
		boolean end = false;
		if (colorSensor instanceof SwitchableLight) {
			((SwitchableLight)colorSensor).enableLight(true);
		}

//            PIDFDrive = new PIDFController(0.0011844, 0.000000, 0.00150719, 0);
//            PIDFStrafe = new PIDFController(0.001705, 0.000000, 0.005705, 0);
//            PIDFTurn = new PIDFController(0.35, 0.00000, 0.395, 0);

//            PIDFDrive = new PIDFController(0.0011844, 0.000000, 0.00150719, 0);
//            PIDFStrafe = new PIDFController(0.001705, 0.000000, 0.005705, 0);
//            PIDFTurn = new PIDFController(0.35, 0.00000, 0.395, 0);

            autoDriving = new AutoDriving(PositionPidfConstants.USDrive, PositionPidfConstants.USStrafe, PositionPidfConstants.USTurn, robot);
            autoDriving.setDefaultMaxVelocity(0.3); //If things don't work start here

            telemetry.addData("Status", "Initialized");
            telemetry.update();


		try {

			if (opModeIsActive() && !isStopRequested()) {
				double startPosx = 1850.85;
				autoDriving.setStartLocation(startPosx,0,0);
				//autoDriving.stopAt(MovementData.withDegrees(startPosX,892.8,90),.3);
				//autoDriving.stopAt(MovementData.withDegrees(1550,892.8,0),.3);

				//autoDriving.driveY(892.8);
				//autoDriving.rotateTo(90);
				//autoDriving.driveX(1550);
				//autoDriving.rotateTo(0);
				autoDriving.stopAt(MovementData.withDegrees(1550,892.8,0),.3);
				red = colorSensor.red();
				if (red > 200){
					telemetry.speak("Zone Owen");
					autoDriving.stopAt(MovementData.withDegrees(startPosx,2900,0),.3);
					//autoDriving.driveY(2900);
				}else if (red > 55){
					telemetry.speak("Zone Bri");
					//autoDriving.stopAt(MovementData.withDegrees(startPosX,2500,90),.3);
					//autoDriving.stopAt(MovementData.withDegrees(1400,2500,0),.3);
					autoDriving.stopAt(MovementData.withDegrees(1400,2500,0),.3);
					autoDriving.rotateTo(90); //Keep if use bottom
					//autoDriving.driveX(1400);
					//autoDriving.rotateTo(0);
					//autoDriving.driveY(2500);
					sleep(1000);
				}else{
					telemetry.speak("Zone Daniel");
					sleep(1000);
					autoDriving.stopAt(MovementData.withDegrees(startPosx,1900,0),.3);
					//autoDriving.driveY(1900);
				}
				lowerWobble.setPosition(Constants.lowerWobbleUp);
				autoDriving.freeDriveXY(-150,-150,.3); //Backs away from wobble
				//autoDriving.freeDriveY(-150);
				autoDriving.stopAt(MovementData.withDegrees(startPosx,2070,0),.3);
				//autoDriving.driveY(2070);



			}
			if (runtime.milliseconds() >= 29000 || end) {
				location = false;
				frontRightMotor.setPower(0);
				frontLeftMotor.setPower(0);
				backRightMotor.setPower(0);

				backLeftMotor.setPower(0);
				telemetry.speak("Done");
				telemetry.update();
				requestOpModeStop();

			}
		} catch (Exception e) {
			telemetry.addData("error:", e.getStackTrace());
			telemetry.addData("bob", location);
			telemetry.update();
			sleep(10000);
		}

	}
}