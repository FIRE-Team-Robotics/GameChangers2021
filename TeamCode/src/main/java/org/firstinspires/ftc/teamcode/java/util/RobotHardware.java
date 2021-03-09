package org.firstinspires.ftc.teamcode.java.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class RobotHardware {

    public BNO055IMU imu;

    public DcMotorEx frontLeftMotor = null;
    public DcMotorEx frontRightMotor = null;
    public DcMotorEx backLeftMotor = null;
    public DcMotorEx backRightMotor = null;

    public DcMotor rightShooter = null;
    public DcMotor leftShooter = null;
    public DcMotor intakeAndDelivery = null;

    public CRServo rShooter;
    public CRServo lShooter;

    public AnalogInput potentiometer = null;
    public TouchSensor wobbleDetector = null;
    public TouchSensor ringCounter = null;

    HardwareMap hardwareMap = null;
    private final ElapsedTime period = new ElapsedTime();

    /**
     * Sets up the HardwareMap
     *
     * @param hardwareMap is the hardware map
     */
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // imu set up parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Parts in hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        //wobbleDetector = hardwareMap.get(TouchSensor.class, "wobbleDetector");
        //ringCounter = hardwareMap.get(TouchSensor.class, "ringCounter");

        lShooter = hardwareMap.get(CRServo.class,"l1" );
        rShooter = hardwareMap.get(CRServo.class,"r1" );
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
//        intakeAndDelivery = hardwareMap.get(DcMotor.class, "intakeAndDelivery");
//        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
//        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");

        // Motor Directions
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        //intakeAndDelivery.setDirection(DcMotor.Direction.FORWARD);
//        rightShooter.setDirection(DcMotor.Direction.FORWARD);
//        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //intakeAndDelivery.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Turn off all motors
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        //intakeAndDelivery.setPower(0);
//        rightShooter.setPower(0);
//        leftShooter.setPower(0);
    }
}
