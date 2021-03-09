package org.firstinspires.ftc.teamcode.java.subsystems.Intake;


import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {

	private final DcMotorEx intakeMotor;

	private IntakeState state = IntakeState.STATIC;

	public Intake(final DcMotorEx intakeMotor) {
		this.intakeMotor = intakeMotor;
	}

	public void set(final IntakeState state) {
		double intakePower;
		switch (state) {
			default:
			case STATIC:
				intakePower = 0;
				break;

			case COLLECTION:
				intakePower = 1;
				break;

			case DEPLETION:
				intakePower = -1;
				break;
		}
		this.state = state;
		intakeMotor.setPower(intakePower);
	}

}
