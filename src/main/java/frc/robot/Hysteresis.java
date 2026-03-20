// FRC Team 4034

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * David's original hysteresis implementation, tuned for swerve drive.
 */
class Hysteresis1 {
	private double last_update_time;
	private double last_input;

	// How fast to pull the joystick to trigger hysteresis.
	private double diff_threshold = 0.1;
	// How fast the speed increases during hysteresis.
	private double increase_speed = 5.5;
	// The multiplier for the speed to make hysteresis work. Should always be a value
	// from 0 to 1. This is the output of `next_multiplier()`. Multiply our joystick
    // input by this value to dampen its magnitude.
	private double hysteresis_mult = 1.0;

	public Hysteresis1() {
		this.last_update_time = Timer.getFPGATimestamp();
	}

	public double next_multiplier(double current_input) {
		double current_time = Timer.getFPGATimestamp();
		double delta_time = current_time - last_update_time;
		// calculate joystick speed
		double input_diff = Math.abs(current_input - last_input);

		if (input_diff > diff_threshold) {
			hysteresis_mult = 0.0;
		}

		if (hysteresis_mult < 1.0) {
			hysteresis_mult += increase_speed * delta_time;
		} else {
			hysteresis_mult = 1.0;
		}

		last_input = current_input;
		last_update_time = current_time;

		return hysteresis_mult;
	}

	public void log() {
		SmartDashboard.putNumber("hysteresis multiplier", hysteresis_mult);
	}
}

/*
 * Sahil's hysteresis implementation from last year. Maps to a quadratic curve.
 */
class Hysteresis2 {
	private double timeToMax;
	private double maxSpeed;
	private double quadradicCoefficient;

	private double currSpeed;
	private double prevTime;

	public Hysteresis2(double timeToMax, double maxSpeed) {
		this.timeToMax = timeToMax;
		this.maxSpeed = maxSpeed;
		this.quadradicCoefficient = this.maxSpeed / Math.pow(this.timeToMax, 2);
        this.reset();
	}

	private void reset() {
		this.currSpeed = 0;
		this.prevTime = Timer.getFPGATimestamp();
	}

	private double timeToSpeed(double time) {
		int sign = time >= 0 ? 1 : -1;
		return quadradicCoefficient * Math.pow(time, 2) * sign;
	}

	private double speedToTime(double speed) {
		int sign = speed >= 0 ? 1 : -1;
		return Math.sqrt((speed * sign) / quadradicCoefficient) * sign;
	}

	public double nextValue(double targetSpeed) {
		double targetDriveTime = speedToTime(targetSpeed);
		double currDriveTime = speedToTime(currSpeed);
		double driveTimeDiff = targetDriveTime - currDriveTime;
		int direction = driveTimeDiff >= 0 ? 1 : -1;

		double currTime = Timer.getFPGATimestamp();
		double actualElapsedTime = currTime - prevTime;

		double driveAdjustTime = Math.min(
			actualElapsedTime, Math.abs(driveTimeDiff) );

		double newTimeForSpeed = currDriveTime + (driveAdjustTime * direction);

		prevTime = currTime;
		currSpeed = timeToSpeed(newTimeForSpeed);
		return currSpeed;
	}

	public void log() {
		SmartDashboard.putNumber("hysteresis2, currSpeed", currSpeed);
	}
}
