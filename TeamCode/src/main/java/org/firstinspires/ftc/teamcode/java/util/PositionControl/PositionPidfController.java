package org.firstinspires.ftc.teamcode.java.util.PositionControl;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * the PIDController class will be used for  all the different pid calculations
 * for example AutoDriving ,autoAdjusting , controlling the shooter speed
 */
//P is main power, I looks at the sum of error and gives final push, D is how much the error is changing
public class PositionPidfController {
	private double maxI = .1;
	private double minI = -.1;

	//TODO look at using System instead of ElapsedTime
	private final ElapsedTime elapsedTime;

	private double previousTime;
	private double kp;
	private double ki;
	private double kd;
	private double f;

	double integral = 0;
	double derivative = 0;

	public PositionPidfController(double kp, double ki, double kd, double f) {
		elapsedTime = new ElapsedTime();
		this.previousTime = 0;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.f = f;
	}

	public double getMaxIntegral() {
		return maxI;
	}

	public void setMaxIntegral(double maxI) {
		this.maxI = maxI;
	}

	public double getMinIntegral() {
		return minI;
	}

	public void setMinIntegral(double minI) {
		this.minI = minI;
	}

	public double getP() {
		return kp;
	}

	public void setP(double kp) {
		this.kp = kp;
	}

	public double getI() {
		return ki;
	}

	public void setI(double ki) {
		this.ki = ki;
	}

	public double getD() {
		return kd;
	}

	public void setD(double kd) {
		this.kd = kd;
	}

	public double getF() {
		return f;
	}

	public void setF(double f) {
		this.f = f;
	}

	public double calculate(double error) {
		double currentTime = elapsedTime.nanoseconds();
		double p = kp * error;
		double i = Range.clip(integral + ki * (error * (currentTime - previousTime)), minI, maxI);
		integral = i;
		double d = kd * ((error - derivative) / (currentTime - previousTime));
		derivative = error;
		previousTime = currentTime;

		return f + (1 - f) * (p + i + d);
	}
}
