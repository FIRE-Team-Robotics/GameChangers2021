package org.firstinspires.ftc.teamcode.java.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.movement.ActiveLocation;
import org.firstinspires.ftc.teamcode.java.movement.AutoDriving;
import org.firstinspires.ftc.teamcode.java.movement.PathFinder;
import org.firstinspires.ftc.teamcode.java.util.PositionControl.PositionPidfController;
import org.firstinspires.ftc.teamcode.java.util.RobotHardware;

//To fix error perhaps flip the switch

@Autonomous(name="ActiveLocationTest", group="Backup")

public class ActiveLocationTest extends LinearOpMode {

    // Declare OpMode members.
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    //Hardware robot = new Hardware();
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private BNO055IMU imu;
    private ActiveLocation activeLocation;
    private Thread locationThread;
    private PathFinder pathFinder;
    private Thread pathThread;
    private AutoDriving autoDriving;
    private PositionPidfController PIDF;

    //private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        imu = robot.imu;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        try {
            //activeLocation.setStartPosition(600,600, 90);
            pathFinder.setDestination(600, 600, 0);
            /*
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }*/
            while (opModeIsActive()) {
                telemetry.addData("X", activeLocation.getFieldX());
                telemetry.addData("Y", activeLocation.getFieldY());
                telemetry.addData("Angle", Math.toDegrees(imu.getAngularOrientation().firstAngle));
                telemetry.addData("Pain", activeLocation.getAngleInDegrees());
                telemetry.addData("Path: ", pathFinder.getEncoderPath());
                telemetry.addData("Raw SPain", pathFinder.getEncoderPath().getAngleInDegrees());

                //telemetry.addData("Error",autoDriving.errorReport(MovementData.withDegrees(600,600,90)));
                telemetry.update();
                sleep(300);
            }

            activeLocation.stop();
            pathFinder.stop();
        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace());
            //AL.Stop();
            //PF.stop();
            telemetry.update();
        }
    }
}
