package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.java.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.java.util.Angle;
import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

/**
 * The AutoDriving class allows the robot to move to specified locations after calculating with
 * PathFinder.
 */
public class AutoDrivingNew {
	/**
	 * {@link PositionPidfController} for the 3 Axes of Movement: X, Y, and θ
	 */
	private final PositionPidfController drivePid, strafePid, turnPid;
	/**
	 * A Location Tracking {@link ActiveLocation} Object to Update the Current Location
	 */
	private final ActiveLocation activeLocation;
	/**
	 * A {@link PathFinder} Object to find the best path to the final Location
	 */
	private final PathFinder pathFinder;
	/**
	 * The {@link Thread} to allow running {@link ActiveLocation} and {@link PathFinder} continuously
	 */
	private final Thread locationThread, pathThread;
	/**
	 * The Main Motors {@link DcMotor} on the Drive train to Allow Movement control
	 */
	private final DcMotorEx frontRight, frontLeft, backRight, backLeft;
	/**
	 * The default Maximum Velocity in case one is not passed when calling a specific
	 * movement
	 */
	private double defaultMaxVelocity = 1.0;
	/**
	 * The default Error Ranges for Translational Movement in case one is not passed through
	 */
	private double defaultErrorX = 40, defaultErrorY = 40;
	/**
	 * The default Error Ranges for Rotational Movement as an {@link Angle} in case it is not passed through
	 */
	private Angle defaultErrorAngle = Angle.fromDegrees(5);

	/**
	 * The Basic Constructor to Create a Basic Instance of AutoDriving
	 * @param drivePid the forward and reverse {@link PositionPidfController}
	 * @param strafePid the left and right {@link PositionPidfController}
	 * @param turnPid the rotation {@link PositionPidfController}
	 * @param robot the {@link RobotHardware} initiated with a HardwareMap
	 */
	public AutoDrivingNew(PositionPidfController drivePid, PositionPidfController strafePid,
	                      PositionPidfController turnPid, RobotHardware robot) {
		// Connects PID Controllers with tuned variants
		this.drivePid   = drivePid;
		this.strafePid  = strafePid;
		this.turnPid    = turnPid;

		// Assigns Hardware values to local copies
		frontRight  = robot.frontRightMotor;
		frontLeft   = robot.frontLeftMotor;
		backLeft    = robot.backLeftMotor;
		backRight   = robot.backRightMotor;

		// Starts an ActiveLocation
		activeLocation = new ActiveLocation(robot);
		locationThread = new Thread(activeLocation);
		locationThread.start();

		// Starts a PathFinder
		pathFinder = new PathFinder(activeLocation);
		pathThread = new Thread(pathFinder);
		pathThread.start();
	}

	/**
	 * The Basic Constructor to Create a Basic Instance of AutoDriving
	 * @param drivePid the forward and reverse {@link PositionPidfController}
	 * @param strafePid the left and right {@link PositionPidfController}
	 * @param turnPid the rotation {@link PositionPidfController}
	 * @param frontRightMotor the Front Right {@link DcMotor} on the Robot
	 * @param frontLeftMotor the Front Left {@link DcMotor} on the Robot
	 * @param backRightMotor the Back Right {@link DcMotor} on the Robot
	 * @param backLeftMotor the Back Left {@link DcMotor} on the Robot
	 * @param xAxisEncoder a {@link DcMotor} which is linked to the X Axis Encoder
	 * @param yAxisEncoder a {@link DcMotor} which is linked to the Y Axis Encoder
	 * @param gyroscope the imu (aka gyroscope) to determine the angle / orientation of the Robot
	 */
	public AutoDrivingNew(PositionPidfController drivePid, PositionPidfController strafePid,
	                      PositionPidfController turnPid, DcMotor frontRightMotor, DcMotor frontLeftMotor,
	                      DcMotor backRightMotor, DcMotor backLeftMotor, DcMotor xAxisEncoder,
	                      DcMotor yAxisEncoder, BNO055IMU gyroscope) {
		// Connects PID Controllers with tuned variants
		this.drivePid   = drivePid;
		this.strafePid  = strafePid;
		this.turnPid    = turnPid;

		// Assigns Hardware values to local copies
		frontRight  = (DcMotorEx) frontRightMotor;
		frontLeft   = (DcMotorEx) frontLeftMotor;
		backLeft    = (DcMotorEx) backLeftMotor;
		backRight   = (DcMotorEx) backRightMotor;

		// Starts an ActiveLocation
		activeLocation = new ActiveLocation(xAxisEncoder, yAxisEncoder, gyroscope);
		locationThread = new Thread(activeLocation);
		locationThread.start();

		// Starts a PathFinder
		pathFinder = new PathFinder(activeLocation);
		pathThread = new Thread(pathFinder);
		pathThread.start();
	}

	/**
	 * Set the start location for the Robot
	 * @param startLocation the new start location
	 */
	public void setStartLocation(MovementData startLocation) {
		activeLocation.setStartPosition(startLocation);
	}

	/**
	 * Update the Default Maximum Velocity to a new user set value
	 * @param defaultMaxVelocity the new default maximum velocity
	 */
	public void setDefaultMaxVelocity(double defaultMaxVelocity) {
		this.defaultMaxVelocity = defaultMaxVelocity;
	}

	/**
	 * Update the default error ranges for all dimensions
	 * @param errorRanges the new error ranges to set as default
	 */
	public void setDefaultErrorRanges(MovementData errorRanges) {
		this.defaultErrorX = errorRanges.getX();
		this.defaultErrorY = errorRanges.getY();
		this.defaultErrorAngle = errorRanges.getAngle();
	}

	/**
	 * Update the Default X Error Range
	 * @param defaultErrorX the new x error range
	 */
	public void setDefaultErrorX(double defaultErrorX) {
		this.defaultErrorX = defaultErrorX;
	}

	/**
	 * Update the default Y Error Range
	 * @param defaultErrorY the new y error range
	 */
	public void setDefaultErrorY(double defaultErrorY) {
		this.defaultErrorY = defaultErrorY;
	}

	/**
	 * Update the default Angle Error Range
	 * @param defaultErrorAngle the new angle error range
	 */
	public void setDefaultErrorAngle(Angle defaultErrorAngle) {
		this.defaultErrorAngle = defaultErrorAngle;
	}

	/**
	 * Scales all wheel speeds to a scaling factor
	 *
	 * @param wheelSpeeds the speeds to scale
	 * @param scaleTo the scaling factor
	 * @param replaceAll whether or not to disregard the acutal values of the variables
	 */
	private void scaleSpeeds(double[] wheelSpeeds, double scaleTo, boolean replaceAll) {
		if (replaceAll) {
			double maxMagnitude = scalarMax(wheelSpeeds);
			// Scales the speeds
			for (int i = 0; i < wheelSpeeds.length; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * scaleTo;
			}
		} else {
			scaleSpeeds(wheelSpeeds, scaleTo);
		}
	}

	/**
	 * Scales all speeds down to a maximum velocity if any is above it
	 *
	 * @param wheelSpeeds the speeds to scale
	 * @param maximumVelocity the maximum velocity
	 */
	private void scaleSpeeds(double[] wheelSpeeds, double maximumVelocity) {
		double maxMagnitude = scalarMax(wheelSpeeds);
		if (maxMagnitude > maximumVelocity) {
			// Scales the speeds
			for (int i = 0; i < wheelSpeeds.length; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * maximumVelocity;
			}
		}
	}

	/**
	 * Scales all wheel speeds so that none are greater than the maximum speed.
	 *
	 * @param wheelSpeeds the speeds to scale
	 */
	private void scaleSpeeds(double[] wheelSpeeds) {
		scaleSpeeds(wheelSpeeds, defaultMaxVelocity);
	}

	/**
	 * Finds the scalar maximum of an array of doubles
	 *
	 * Scalar Maximum means that only the magnitude, not the sign, will be taken into account
	 * in the calculation
	 * @param values an array of values from which the maximum must be found
	 * @return the maximum
	 * @throws IllegalArgumentException protects against arrays with no elements
	 */
	private double scalarMax(double[] values) throws IllegalArgumentException {
		if (values.length == 0)
			throw new IllegalArgumentException("There must be at least one value in the array");

		double scalarMax = Math.abs(values[0]);
		for (int i = 1; i < values.length; i++) {
			scalarMax = Math.max(scalarMax, Math.abs(values[i]));
		}
		return scalarMax;
	}

	/**
	 * Finds the vector maximum of an array of doubles
	 *
	 * Vector Maximum means that both the direction (positive/negative) and the magnitude are taken
	 * into account, where positive numbers are all greater than negative numbers.
	 *
	 * @param values an array of values from which the maximum must be found
	 * @return the maximum
	 * @throws IllegalArgumentException protects against arrays with no elements
	 */
	private double vectorMax(double[] values) throws IllegalArgumentException {
		if (values.length == 0)
			throw new IllegalArgumentException("There must be at least one value in the array");

		double vectorMax = values[0];
		for (int i = 1; i < values.length; i++) {
			vectorMax = Math.max(vectorMax, values[i]);
		}
		return vectorMax;
	}

	/**
	 * Checks if the Robot is within reasonable error to its goal position
	 * @param goal the goal position for the robot to reach
	 * @param errorX the reasonable error range for the Δx position with respect to the field
	 * @param errorY the reasonable error range for the Δy position with respect to the field
	 * @param errorAngle the reasonable error range for the Δθ position with respect to the
	 * @return whether or not the robot is within a reasonable error range
	 */
	private boolean arrivedAt(MovementData goal, double errorX, double errorY, Angle errorAngle) {
		return  Math.abs(activeLocation.getFieldX() - goal.getX()) < errorX &&
				Math.abs(activeLocation.getFieldY() - goal.getY()) < errorY &&
				Math.abs(
						activeLocation.getAngle() - goal.getAngleInRadians()
				) < errorAngle.getAngleInRadians();
	}

	/**
	 * Checks if the Robot is within reasonable error to its goal position
	 * @param goal the goal position for the robot to reach
	 * @param errorRange the reasonable established error range
	 * @return whether or not the robot is within a reasonable error range
	 */
	private boolean arrivedAt(MovementData goal, MovementData errorRange) {
		return arrivedAt(goal, errorRange.getX(), errorRange.getY(), errorRange.getAngle());
	}

	/**
	 * Checks if the Robot is within reasonable error to its goal position, using the default as the
	 * reasonable error
	 * @param goal the goal position for the robot to reach
	 * @return whether or not the robot is within a reasonable error range
	 */
	private boolean arrivedAt(MovementData goal) {
		return arrivedAt(goal, defaultErrorX, defaultErrorY, defaultErrorAngle);
	}

	/**
	 * Calculates Mecanum Wheel Drive Powers with PID
	 * @param maxVelocity the maximum velocity
	 * @param xError the horizontal axis error
	 * @param yError the vertical axis error
	 * @param angleError the rotational error
	 * @return an array with the powers for the wheels
	 */
	public double[] calculateDrivePowers(double maxVelocity, double xError, double yError, double angleError) {
		double strafePower = strafePid.calculate(xError);
		double drivePower = drivePid.calculate(yError);
		double turnPower = turnPid.calculate(angleError);

		double[] wheelSpeeds = MecanumDrive.calculateDrivePowers(
				drivePower, strafePower, turnPower
		);

		scaleSpeeds(wheelSpeeds, maxVelocity);

		return wheelSpeeds;
	}

	/**
	 * Calculates Mecanum Wheel Drive Powers with PID using the default maximum velocity
	 * @param xError the horizontal axis error
	 * @param yError the vertical axis error
	 * @param angleError the rotational error
	 * @return an array with the powers for the wheels
	 */
	public double[] calculateDrivePowers(double xError, double yError, double angleError) {
		return calculateDrivePowers(defaultMaxVelocity, xError, yError, angleError);
	}

	/**
	 * Calculates Mecanum Wheel Drive Powers with PID
	 * @param maxVelocity the maximum velocity
	 * @param error the error values on all axes
	 * @return an array with the powers for the wheels
	 */
	public double[] calculateDrivePowers(double maxVelocity, MovementData error) {
		return calculateDrivePowers(
				maxVelocity, error.getX(), error.getY(), error.getAngleInRadians()
		);
	}

	/**
	 * Calculates Mecanum Wheel Drive Powers with PID using the default maximum velocity
	 * @param error the error values on all axes
	 * @return an array with the powers for the wheels
	 */
	public double[] calculateDrivePowers(MovementData error) {
		return calculateDrivePowers(defaultMaxVelocity, error);
	}

	/**
	 * Sets the motors to move at the given speeds
	 * @param speeds the speeds to move at
	 */
	public void setMotorPowers(double[] speeds) {
		frontLeft.setPower(speeds[0]);
		frontRight.setPower(speeds[1]);
		backLeft.setPower(speeds[2]);
		backRight.setPower(speeds[3]);
	}

	/**
	 * Stops the Robot from Moving
	 */
	public void turnOff() {
		frontLeft.setPower(0);
		frontRight.setPower(0);
		backLeft.setPower(0);
		backRight.setPower(0);
	}

	/**
	 * Moves the robot to a given point using a PID Loop
	 * @param goal the position the robot should reach
	 * @param maxVelocity the maximum rotational movement of the wheel to reach
	 * @param errorRange the error range of movement
	 * @return false if the robot is already at the goal position, true if the robot has just
	 *         reached the new position
	 */
	public boolean stopAt(MovementData goal, double maxVelocity, MovementData errorRange) {
		if (arrivedAt(goal, errorRange)) return false;
		pathFinder.setDestination(goal);
		while (!arrivedAt(goal, errorRange)) {
			pathFinder.updateEncoderPath();
			setMotorPowers(calculateDrivePowers(maxVelocity, pathFinder.getEncoderPath()));
		}
		//turnOff();
		return true;
	}

	/**
	 * Moves the robot to a given point using a PID Loop
	 * @param goal the position the robot should reach
	 * @param maxVelocity the maximum rotational movement of the wheel to reach
	 * @param xErrorRange the error range for the horizontal direction
	 * @param yErrorRange the error range for the vertical direction
	 * @param angleErrorRange the error range for the rotational axis
	 * @return false if the robot is already at the goal position, true if the robot has just
	 *         reached the new position
	 */
	public boolean stopAt(MovementData goal, double maxVelocity, double xErrorRange,
	                      double yErrorRange, Angle angleErrorRange) {
		return stopAt(
				goal, maxVelocity, new MovementData(xErrorRange, yErrorRange, angleErrorRange)
		);
	}

	/**
	 * Moves the robot to a given point using a PID Loop using the default errors
	 * @param goal the position the robot should reach
	 * @param maxVelocity the maximum rotational movement of the wheel to reach
	 * @return false if the robot is already at the goal position, true if the robot has just
	 *         reached the new position
	 */
	public boolean stopAt(MovementData goal, double maxVelocity) {
		return stopAt(goal, maxVelocity, defaultErrorX, defaultErrorY, defaultErrorAngle);
	}

	/**
	 * Moves the robot to a given point using a PID Loop using the default velocity
	 * @param goal the position the robot should reach
	 * @param xErrorRange the error range for the horizontal direction
	 * @param yErrorRange the error range for the vertical direction
	 * @param angleErrorRange the error range for the rotational axis
	 * @return false if the robot is already at the goal position, true if the robot has just
	 *         reached the new position
	 */
	public boolean stopAt(MovementData goal, double xErrorRange, double yErrorRange,
	                      Angle angleErrorRange) {
		return stopAt(goal, defaultMaxVelocity, xErrorRange, yErrorRange, angleErrorRange);
	}

	/**
	 * Moves the robot to a given point using a PID Loop using the default errors and velocity
	 * @param goal the position the robot should reach
	 * @return false if the robot is already at the goal position, true if the robot has just
	 *         reached the new position
	 */
	public boolean stopAt(MovementData goal) {
		return stopAt(goal, defaultMaxVelocity);
	}

	/**
	 * Rotates the robot to a new {@link Angle}, maintaining the other field positions
	 * @param angle the goal angle to reach
	 * @param maxVelocity the maximum velocity to reach
	 * @param xErrorRange the permitted error range for the horizontal axis
	 * @param yErrorRange the permitted error range for the vertical axis
	 * @param angleErrorRange the permitted error range for the angle
	 * @return whether or not the robot needed to move
	 */
	public boolean rotateTo(Angle angle, double maxVelocity, double xErrorRange, double yErrorRange,
	                        Angle angleErrorRange) {
		return stopAt(
				new MovementData(activeLocation.getFieldX(), activeLocation.getFieldY(), angle),
				maxVelocity,
				xErrorRange,
				yErrorRange,
				angleErrorRange
		);
	}

	/**
	 * Rotates the robot to a new {@link Angle}, maintaining the other field positions
	 * @param angle the goal angle to reach
	 * @param maxVelocity the maximum velocity to reach
	 * @param angleErrorRange the permitted error range for the angle
	 * @return whether or not the robot needed to move
	 */
	public boolean rotateTo(Angle angle, double maxVelocity, Angle angleErrorRange) {
		return rotateTo(angle, maxVelocity, defaultErrorX, defaultErrorY, angleErrorRange);
	}

	/**
	 * Rotates the robot to a new {@link Angle}, maintaining the other field positions
	 * @param angle the goal angle to reach
	 * @param maxVelocity the maximum velocity to reach
	 * @return whether or not the robot needed to move
	 */
	public boolean rotateTo(Angle angle, double maxVelocity) {
		return rotateTo(angle, maxVelocity, defaultErrorX, defaultErrorY, defaultErrorAngle);
	}
}
