package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
import org.firstinspires.ftc.teamcode.java.util.Side;

@Autonomous(name = "Auto Adjusting Test", group = "Testing")
public class AutoAdjustingTest extends LinearOpMode {

	RobotHardware robot = new RobotHardware();
	private BNO055IMU imu;
	private ActiveLocation AL;
	private Thread locationThread;
	private AutoAdjusting autoAdjusting;
	private CRServo pl;
	private CRServo nl;

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);
		imu = robot.imu;
		pl = robot.lShooter;
		nl = robot.rShooter;

		AL = new ActiveLocation(robot);
		locationThread = new Thread(AL);
		locationThread.start();
		autoAdjusting = new AutoAdjusting(robot, AL, Side.RED, this);

		telemetry.addData("Status", "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
		telemetry.update();

		waitForStart();


		telemetry.addData("Test", "1");
		telemetry.update();
		int movement = 1;
		while (opModeIsActive()) {
			if (movement == 1) {
				telemetry.addData("Test", "qwertyuytresdcvbhjuytrdbfghtytfvcbvbmgjhfgbfng");
				telemetry.update();
				sleep(100);
				}
			movement = 2;
			double a = autoAdjusting.getShooterTurnPower();
			pl.setPower(a);
			nl.setPower(a);

		}


	}
}
