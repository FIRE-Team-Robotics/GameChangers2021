package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.java.util.Goal;
import org.firstinspires.ftc.teamcode.java.util.GoalPosition;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;
import org.firstinspires.ftc.teamcode.java.util.Side;

import static org.firstinspires.ftc.teamcode.java.util.MathUtil.squared;

/**
 * Allows the automatic adjustment of the Robot towards the shooter
 */
public class AutoAdjusting {

	private static final double shooterLength   = 420.69; // Temporary Value
	private static final double velocityGoal    = 420.69; // Temporary Value
	public  static final Goal initialGoal       = Goal.POWER_SHOT_1;

	private final AnalogInput       potentiometer;
	private final BNO055IMU         imu;

	private final ActiveLocation    activeLocation;

	private final PositionPidfController PidfYaw, PidfPitch;

	public double deltaX            = 0;
	public double deltaY            = 0;
	public double deltaZ            = 0;

	private double shooterRotationPower = 0;
	private double robotTurnPower = 0;

	private final Side side;
	private GoalPosition activeGoal;
	private int goalPositionIndex = 0;

	Telemetry telemetry;
	LinearOpMode opMode;

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side, LinearOpMode opmode) {
		this(robot, activeLocation, side);
		this.telemetry = opmode.telemetry;
		this.opMode = opmode;
	}

	public AutoAdjusting(RobotHardware robot, ActiveLocation activeLocation, Side side) {
		this.potentiometer  = robot.potentiometer;
		this.imu            = robot.imu;

		this.activeLocation = activeLocation;

		// TODO: Calibrate turn and Pitch Values
		this.PidfYaw    = new PositionPidfController(0.35, 0.00000, 0.395, 0);
		this.PidfPitch  = new PositionPidfController(1,0,0,1);

		this.side = side;
		activeGoal = GoalPosition.generate(side, initialGoal);
	}

	private void update() {
		deltaX = activeGoal.xPosition - activeLocation.getFieldX();
		deltaY = activeGoal.yPosition - activeLocation.getFieldY();
		deltaZ = activeGoal.height    - getCurrentHeight();
		telemetry.addData("Delta Z", deltaZ);
		telemetry.addData("Current Height", getCurrentHeight());
		telemetry.update();
		opMode.sleep(1000);
	}

	public void cycleRight() {
		goalPositionIndex = activeGoal.side == Side.RED
				? (goalPositionIndex + 1) % Goal.values().length
				: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
		activeGoal = GoalPosition.generate(side, Goal.values()[goalPositionIndex]);
	}

	public void cycleLeft() {
		goalPositionIndex = activeGoal.side == Side.BLUE
				? (goalPositionIndex + 1) % Goal.values().length
				: (goalPositionIndex + Goal.values().length - 1) % Goal.values().length;
		activeGoal = GoalPosition.generate(side, Goal.values()[goalPositionIndex]);
	}

	/**
	 * adjusting the pitch angle (using PIDF)
	 */
	public void calculatePitchPower() {
		update();
		shooterRotationPower = PidfPitch.calculate(deltaZ);
	}

	/**
	 * adjusting the yaw angle (using PID)
	 * <p>
	 *           X
	 *     — — — — — — — S
	 *    |
	 *    |
	 * Y  |
	 *    |
	 *    |
	 *    |
	 * <p>
	 * Calculates the angle between the Robot and the Goal and returns the Necessary Adjustment
	 */
	public void calculateYawPower() {
		update();
		double currentAngle = activeLocation.getAngle();
		double yaw          = Math.atan2(deltaX, deltaY);
		double error        = currentAngle - yaw;
		robotTurnPower      = PidfYaw.calculate(error);
	}

	public double getShooterPitchAngle() {
		return (potentiometer.getVoltage() * 81.8);
	}

	private double getCurrentHeight() {
		double distance = Math.sqrt(squared(deltaX) + squared(deltaY));
		double theta = getShooterPitchAngle();
		return  Math.tan(theta) * distance
				- imu.getGravity().zAccel * distance / (2 * squared(velocityGoal) * squared(Math.cos(theta))) // TODO: Figure out IMU Axis
				+ shooterLength * Math.sin(theta);
	}

	public double getTurnPower() {
		calculateYawPower();
		return robotTurnPower;
	}

	public double getShooterTurnPower() {
		calculatePitchPower();
		return shooterRotationPower;
	}
}

