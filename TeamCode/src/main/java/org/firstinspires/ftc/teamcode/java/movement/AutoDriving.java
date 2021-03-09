package org.firstinspires.ftc.teamcode.java.movement;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.java.util.MovementData;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

import java.util.Locale;

/**
 * The AutoDriving class allows the robot to move to specified locations after calculating with
 * PathFinder.
 */
//TODO: Create a tuning class
public class AutoDriving {

	private final PositionPidfController PIDFDrive;
	private final PositionPidfController PIDStrafe;
	private final PositionPidfController PIDFTurn;
	private final ActiveLocation activeLocation;
	private final DcMotorEx frontRightMotor;
	private final DcMotorEx frontLeftMotor;
	private final DcMotorEx backLeftMotor;
	private final DcMotorEx backRightMotor;
	private final Thread locationThread;
	private final PathFinder pathFinder;
	private final Thread pathThread;
	private double defaultMaxVelocity = 0;
	private MovementData defaultErrorRanges;

	private double defaultErrorX = 40;
	private double defaultErrorY = 40;
	private double defaultErrorAngle = 5;
	RobotHardware robot;

	// TODO: Either make Robot Hardware somehow implementable in the library or have a manual addition of the motors
    public AutoDriving(PositionPidfController PIDFDrive, PositionPidfController PIDFStrafe, PositionPidfController PIDFTurn, RobotHardware robot) {
        this.PIDFDrive = PIDFDrive;
        this.PIDStrafe = PIDFStrafe;
        this.PIDFTurn = PIDFTurn;
        this.robot = robot;
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor = robot.backRightMotor;
        backLeftMotor = robot.backLeftMotor;

        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();

        pathFinder = new PathFinder(activeLocation);
        pathThread = new Thread(pathFinder);
        pathThread.start();
    }

    public void setDefaultMaxVelocity(double defaultMaxVelocity){
        this.defaultMaxVelocity = defaultMaxVelocity;
    }
    public void setDefaultErrorRanges(MovementData errorRange){
    	this.defaultErrorX = errorRange.getX();
    	this.defaultErrorY = errorRange.getY();
    	this.defaultErrorAngle = errorRange.getAngleInDegrees();
    }
    public void setDefaultErrorRanges(double xErrorRange, double yErrorRange, double angleErrorRange){
    	setDefaultErrorRanges(MovementData.withDegrees(xErrorRange,yErrorRange,angleErrorRange));
    }
    public void setDefaultDriveErrorRange(double errorRange){
    	setDefaultErrorRanges(errorRange,errorRange,defaultErrorAngle);

    }
    public void setDefaultAngleErrorRange(double errorRange){
		setDefaultErrorRanges(defaultErrorX,defaultErrorY,errorRange);
    }
    public void setDefaultErrorRangeX(double errorRange){
    	setDefaultErrorRanges(errorRange, defaultErrorY, defaultErrorAngle);
    }
    public void setDefaultErrorRangeY(double errorRange){
    	setDefaultErrorRanges(defaultErrorX, errorRange, defaultErrorAngle);
    }

    //TODO fill blank functions write aPID controller  add more functionality

    /**
     * drives to a point and stops using PID
     */
    //TODO implement false return if already at position
    public boolean stopAt(MovementData goal, double maxVelocity, MovementData errorRanges) {
    	double errorRangeX = errorRanges.getX();
    	double errorRangeY = errorRanges.getY();
    	double errorRangeAngle = errorRanges.getAngleInDegrees();

        boolean arrived = false;
        while (!arrived) {
            pathFinder.setDestination(goal);
            MovementData error = pathFinder.getEncoderPath();
            double[] speeds = calculateDrivePowers(maxVelocity, error);//PIDFDrive.calculateDrivePowers(maxVelocity, error);
            setMotorPowers(speeds);
            if ((Math.abs(goal.getX() - activeLocation.getFieldX()) < errorRangeX) && (Math.abs(goal.getY() - activeLocation.getFieldY()) < errorRangeY) && (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= errorRangeAngle)) {
                arrived = true;
            }
        }
        turnOff();
        return arrived;
    }
    public boolean stopAt(MovementData goal){
        return stopAt(goal, defaultMaxVelocity, MovementData.withDegrees(defaultErrorX,defaultErrorY,defaultErrorAngle));
    }
	public boolean stopAt(MovementData goal, MovementData errorRanges){
		return stopAt(goal, defaultMaxVelocity, errorRanges);
	}
	public boolean stopAt(MovementData goal, double VMax){
		return stopAt(goal, VMax, MovementData.withDegrees(defaultErrorX,defaultErrorY,defaultErrorAngle));
	}
	public boolean stopAt(MovementData goal, double VMax, double xErrorRange, double yErrorRange, double angleErrorRange){
		return stopAt(goal, VMax, MovementData.withDegrees(xErrorRange,yErrorRange,angleErrorRange));
	}
//	public boolean stopAt(MovementData goal, double xErrorRange, double yErrorRange, double angleErrorRange){
//		return stopAt(goal, defaultMaxVelocity, MovementData.withDegrees(xErrorRange,yErrorRange,angleErrorRange));
//	}
	public boolean stopAt(MovementData goal, double VMax, double driveErrorRange, double angleErrorRange){
		return stopAt(goal, VMax, MovementData.withDegrees(driveErrorRange,driveErrorRange,angleErrorRange));
	}

    public double[] calculateDrivePowers(double maxVelocity, MovementData errors) {
        return calculateDrivePowers(maxVelocity, errors.getX(), errors.getY(), errors.getAngleInRadians());
    }

    public double[] calculateDrivePowers(double maxVelocity, double xError, double yError, double angleError) {
        double strafe = PIDStrafe.calculate(xError);
        double drive = PIDFDrive.calculate(yError);
        double twist = PIDFTurn.calculate(angleError);
        //double twist = 0;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (Math.abs(speed) > max) {
                max = Math.abs(speed);
            }
        }
        if (max > maxVelocity) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] *= maxVelocity / max;
            }
        }

        return speeds;
    }

    /**
     * drives through a point without stopping
     */
    public  void passAt() {
    }

    /**
     * rotates to an angle and keeps it using PID
     */
    public boolean rotateTo(double angle, double maxVelocity) {
        MovementData goal = MovementData.withDegrees(activeLocation.getFieldX(),activeLocation.getFieldY(),angle);
        return stopAt(goal,maxVelocity);
    }
    public boolean rotateTo(double angle){
        return rotateTo(angle, defaultMaxVelocity);
    }
    /**
     * uses all the above functions for combination of driving and tuning
     */
    public boolean driveTo(double x, double y, double maxVelocity) {
        MovementData goal = MovementData.withDegrees(x,y,activeLocation.getAngleInDegrees());
        return stopAt(goal, maxVelocity);
    }
    public boolean driveXY(double x, double y, double maxVelocity){
        MovementData goal = MovementData.withDegrees((activeLocation.getFieldX()+x),(activeLocation.getFieldY()+y), activeLocation.getAngleInDegrees());
        return stopAt(goal,maxVelocity);
    }
    
    public boolean driveTo(double x, double y){
        return driveTo(x,y, defaultMaxVelocity);
    }

    public boolean driveX(double x, double maxVelocity) {
        MovementData goal = MovementData.withDegrees(x,activeLocation.getFieldY(),activeLocation.getAngleInDegrees());
        return stopAt(goal,maxVelocity);
    }
    public boolean driveX(double x){
        return driveX(x, defaultMaxVelocity);
    }
    public boolean driveY(double y, double maxVelocity) {
        MovementData goal = MovementData.withDegrees(activeLocation.getFieldX(),y,activeLocation.getAngleInDegrees());
        return stopAt(goal,maxVelocity);
    }
    public boolean driveY(double y){
        return driveY(y, defaultMaxVelocity);
    }

    public boolean freeRotate(double angle, double maxVelocity) {
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(0, 0, angle);
        freeMove(goal,maxVelocity, 2);
        turnOff();
        return arrived;
    }
    public boolean freeRotate(double angle){
        return freeRotate(angle, defaultMaxVelocity);
    }

    public boolean freeDriveX(double x, double maxVelocity){
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(x, 0, 0);
        freeMove(goal,maxVelocity, 0);
        turnOff();
        return arrived;
    }
    public boolean freeDriveX(double x){
        return freeDriveX(x, defaultMaxVelocity);
    }
    public boolean freeDriveY(double y, double maxVelocity){
        boolean arrived = false; // TODO Arrive is not changing
        MovementData goal = MovementData.withDegrees(0, y, 0);
        freeMove(goal,maxVelocity, 0);
        turnOff();
        return arrived;
    }
    public boolean freeDriveY(double y){
        return freeDriveY(y, defaultMaxVelocity);
    }
    public boolean freeDriveXY(double x, double y, double maxVelocity){
        boolean arrived = false;
        MovementData goal = MovementData.withDegrees(x, y, 0);
        freeMove(goal,maxVelocity,7); //Use large number to get default
        turnOff();
        return arrived;
    }
    public boolean freeDriveXY(double x, double y){
        return freeDriveXY(x,y, defaultMaxVelocity);
    }

    private boolean freeMove(MovementData goal, double maxVelocity, int which){ //Keep private/make others like this private
        boolean arrived = false;
        while (!arrived) {
            pathFinder.setDestination(goal);
            MovementData error = pathFinder.getEncoderPath();
            double[] speeds = calculateDrivePowers(maxVelocity, goal);//PIDFDrive.calculateDrivePowers(maxVelocity, error);
            setMotorPowers(speeds);
            switch (which) {
                case 0:
                    if (Math.abs(goal.getX() - activeLocation.getFieldX()) < 40) {
                        arrived = true;
                    }
                    break;
                case 1:
                    if (Math.abs(goal.getY() - activeLocation.getFieldY()) < 40) {
                        arrived = true;
                    }
                    break;
                case 2:
                    if (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 25) {
                        arrived = true;
                    }
                    break;
                default:
                    if ((Math.abs(goal.getX() - activeLocation.getFieldX()) < 40) && (Math.abs(goal.getY() - activeLocation.getFieldY()) < 40) && (Math.abs(pathFinder.getEncoderPath().getAngleInDegrees()) <= 25)) {
                        arrived = true;
                    }
            }

        }
        return arrived;
    }

    public void turnOff(){
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    /**
     * Sets start location in mm and degrees
     * @param location location you are starting at a MovementData
     */
    public void setStartLocation(MovementData location){
        activeLocation.setStartPosition(location);

    }
    public void setStartLocation(double x, double y, double angle){
        this.setStartLocation(MovementData.withDegrees(x,y,angle));
    }


    public String errorReport(MovementData goal) {
        return String.format(
                Locale.ENGLISH,
                "X: %.2f Y: %.2f A: %.2f",
                Math.abs(goal.getX() - activeLocation.getFieldX()),
                Math.abs(goal.getY() - activeLocation.getFieldY()),
                - pathFinder.getEncoderPath().getAngleInDegrees()
        );

    }
    public String errorReport(double x,double y,double z){
        return this.errorReport(MovementData.withDegrees(x,y,z));
    }


    private void setMotorPowers(double[] speeds) {
        frontLeftMotor.setPower(speeds[0]);
        frontRightMotor.setPower(speeds[1]);
        backLeftMotor.setPower(speeds[2]);
        backRightMotor.setPower(speeds[3]);
    }
}
