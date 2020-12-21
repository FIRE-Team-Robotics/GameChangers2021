package org.firstinspires.ftc.teamcode.java.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class Hardware{

    public BNO055IMU imu;

    public DcMotor frontLeftMotor =null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor rightShooter = null;
    public DcMotor leftShooter = null ;
    public DcMotor intakeAndDelivery = null;




    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /**
     * Sets up the HardwareMap
     * @param hwMap is the hardware map
     */
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        //imu set up parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "BNO055IMUCalibration.json";
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Parts in hardware map
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        frontLeftMotor  = hwMap.get(DcMotor.class, "frontLeftMotor" );
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor   = hwMap.get(DcMotor.class, "backLeftMotor"  );
        backRightMotor  = hwMap.get(DcMotor.class, "backRightMotor" );

        intakeAndDelivery  = hwMap.get(DcMotor.class, "intakeAndDelivery" );
        rightShooter = hwMap.get(DcMotor.class, "rightShooter");
        leftShooter = hwMap.get(DcMotor.class, "leftShooter");


        //Motor Directions
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection (DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection (DcMotor.Direction.REVERSE);

        intakeAndDelivery.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeAndDelivery.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Turn off all motors
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower (0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower (0);
        intakeAndDelivery.setPower(0);
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }
}