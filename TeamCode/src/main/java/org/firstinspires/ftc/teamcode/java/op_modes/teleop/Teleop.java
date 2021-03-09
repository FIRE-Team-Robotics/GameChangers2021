package org.firstinspires.ftc.teamcode.java.op_modes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
//import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.movement.AutoAdjusting;
import org.firstinspires.ftc.teamcode.java.util.*;

@TeleOp(name = "Final TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    //private DcMotor intakeAndDelivery;
    private DcMotor leftShooter;
    private DcMotor rightShooter;
    //public RevColorSensorV3 colorSensor;
    private DcMotor elevator;
    private Servo ringArm;

    private Servo lowerWobble;
    //private TouchSensor wobbleDetector;
    //private TouchSensor ringCounter;

    RobotHardware robot = new RobotHardware();

    // TODO change the max speed to 1

    private double maxSpeed = 0.7;


    private double drive = 0;
    private double strafe = 0;
    private double twist = 0;
    private final double intakeAndDeliveryPower = 0;
    private final double shooterPower = 0;
    private final int rings = 0;

    private final boolean ifReversedIntakePressed = false;
    private final boolean shooterIsPressed = false;
    private boolean slowModePressed = false;
    private boolean slowMode = false;


    private AutoAdjusting autoAdjusting;
    private ActiveLocation activeLocation;
    Thread locationThread;

    int red;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        frontLeftMotor = robot.frontLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backLeftMotor = robot.backLeftMotor;
        backRightMotor = robot.backRightMotor;

        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        elevator.setDirection(DcMotor.Direction.FORWARD);
        //elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ringArm = hardwareMap.get(Servo.class, "ringArm");


        //colorSensor = hardwareMap.get(RevColorSensorV3.class,"colorSensor");
       // intakeAndDelivery = robot.intakeAndDelivery;
        //rightShooter = robot.rightShooter;
        //leftShooter = robot.leftShooter;
        //lowerWobble =robot.lowerWobble;
        //wobbleDetector =robot.wobbleDetector;
        //ringCounter =robot.ringCounter;


        activeLocation = new ActiveLocation(robot);
        locationThread = new Thread(activeLocation);
        locationThread.start();
        //autoAdjusting = new AutoAdjusting(robot);

        waitForStart();
        try {
            //TODO mode change and angel reset and shooter adjusting

            while (opModeIsActive()) {
                //motors powers calculation

                drive = -gamepad1.left_stick_y * Math.cos(activeLocation.getAngle()) -
                        gamepad1.left_stick_x * Math.sin(activeLocation.getAngle());
                strafe = gamepad1.left_stick_x * Math.cos(activeLocation.getAngle()) +
                        -gamepad1.left_stick_y * Math.sin(activeLocation.getAngle());
                twist = -gamepad1.right_stick_x;

                // wheel speed calculation
                double[] speeds = {
                        (drive + strafe + twist),
                        (drive - strafe - twist),
                        (drive - strafe + twist),
                        (drive + strafe - twist)
                };

                // Finds the max after converting doubles to Doubles
                double max = Math.abs(speeds[0]);
                for (double speed : speeds) {
                    if (Math.abs(speed) > max) {
                        max = Math.abs(speed);
                    }
                }
                //double max = Math.abs(Collections.max(Arrays.stream(speeds).boxed().collect(Collectors.toList())));

                if (max > maxSpeed) {
                    for (int i = 0; i < speeds.length; i++) speeds[i] *= maxSpeed / max;
                }

                // slow mode
                if (gamepad1.a && !slowMode && !slowModePressed) {
                    slowModePressed = true;
                    slowMode = true;
                    maxSpeed = 0.3;
                } else if (gamepad1.a && slowMode && !slowModePressed) {
                    slowModePressed = true;
                    slowMode = false;
                    maxSpeed = 0.7;
                } else if (!gamepad1.a) {
                    slowModePressed = false;
                }
                //Lower wobble movement
                if (gamepad1.right_bumper){
                    if (lowerWobble.getPosition()>=(Math.abs(Constants.lowerWobbleUp-.2))){
                        lowerWobble.setPosition(Constants.lowerWobbleDown);
                    }else{
                        lowerWobble.setPosition(Constants.lowerWobbleUp);
                    }

                }

//                red = colorSensor.red();
//
//                if (red > 200){
//                    telemetry.speak("Zone Owen");
//                }else if (red > 55){
//                    telemetry.speak("Zone Bri");
//                }else{
//                    telemetry.speak("Zone Daniel");
//                }
                //sleep(1000);

//                if (rings<3){
//                    intakeAndDeliveryPower = gamepad2.left_trigger;
//                    if (ringCounter.isPressed()){
//                        rings++;
//                    }
//                }else if (gamepad2.right_bumper)
//                {
//                    rings=0;
//                }else {
//                    intakeAndDeliveryPower=0;
//                }


                // Angle Resetting
                if (gamepad1.start) {
                    activeLocation.resetAngle();
                }
                /*
                // reversing the intake and delivery
                if (gamepad2.back && !ifReversedIntakePressed) {
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.REVERSE);
                    ifReversedIntakePressed = true;
                } else if (!gamepad2.back) {
                    ifReversedIntakePressed = false;
                    intakeAndDelivery.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                telemetry.speak("I am g");
                telemetry.update();
                sleep(1000);
                //turning on the shooter

                if (gamepad2.a && !shooterIsPressed) {
                    shooterIsPressed = true;
                    shooterPower = 1;
                } else if (!gamepad2.a) {
                    shooterIsPressed = false;
                    shooterPower = 0;
                }
                telemetry.speak("I am root");
                telemetry.update();
                sleep(1000);
                */
                if (gamepad1.dpad_up){
                    elevator.setPower(0.3);
                    telemetry.speak("Yes");
                }else if (gamepad1.dpad_down){
                    elevator.setPower(-0.3);
                }
                else{
                    elevator.setPower(0);
                }
                if (gamepad1.left_bumper){
                    if (ringArm.getPosition()>.4){
                        ringArm.setPosition(0);
                    }else{
                        ringArm.setPosition(1);
                    }
                }
                //setting the speed to the motors
                frontLeftMotor.setPower(speeds[0]);
                frontRightMotor.setPower(speeds[1]);
                backLeftMotor.setPower(speeds[2]);
                backRightMotor.setPower(speeds[3]);
                //intakeAndDelivery.setPower(intakeAndDeliveryPower);
                //leftShooter.setPower(shooterPower);
                // rightShooter.setPower(shooterPower);
                telemetry.addData("Red", red);
                telemetry.addData("FL", frontLeftMotor.getPower());
                telemetry.addData("FR", frontRightMotor.getPower());
                telemetry.addData("BL", backLeftMotor.getPower());
                telemetry.addData("BR", backRightMotor.getPower());
                telemetry.addData("sPAIN", activeLocation.getAngleInDegrees());
                telemetry.update();
                //telemetry.addData("field X:", activeLocation.getFieldX());
                //telemetry.addData("field Y:", activeLocation.getFieldY());
                //telemetry.addData("potentiometer", autoAdjusting.getShooterPitchAngle());
                //telemetry.addData("angle:", activeLocation.getAngleInDegrees());
                //telemetry.addData("rings", rings);
                //telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            telemetry.update();
            sleep(2000);
            activeLocation.stop();
            requestOpModeStop();
        }
    }
}
