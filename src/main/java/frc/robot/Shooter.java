// FRC Team 4034

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ConfigConsts;

public class Shooter {
	public enum ShootingState {
		NOTHING,
		LAUNCH,
		UNLAUNCH;
	}

	private TalonSRX shooterIn;
	private TalonFX shooterOut;

	private double shootInLaunchSpeed = -1;
	private double shootInUnlaunchSpeed = 0.25;

	private double shootOutLaunchSpeed = -0.75;
	private double shootOutUnlaunchSpeed = 0.5;

	// Slow speeds for testing
	// private double shootInLaunchSpeed = -0.25;
	// private double shootInUnlaunchSpeed = 0.25;
	//
	// private double shootOutLaunchSpeed = -0.1;
	// private double shootOutUnlaunchSpeed = 0.1;

	private ShootingState currentOverallState = ShootingState.NOTHING;
	double timeAtPress = 0.0;

	public Shooter() {
		this.shooterIn = new TalonSRX( ConfigConsts.shooterInMotorId );
		this.shooterOut = new TalonFX( ConfigConsts.shooterOutKrakenMotorId );
	}

	public void setShootingState(ShootingState newOverallState) {
		if (newOverallState == currentOverallState) {
			return;
		}
		currentOverallState = newOverallState;
		timeAtPress = Timer.getFPGATimestamp();
	}

	public void periodic() {
		ShootingState shooterInState = ShootingState.NOTHING;
		ShootingState shooterOutState = ShootingState.NOTHING;
		if (currentOverallState == ShootingState.LAUNCH) {
			shooterOutState = ShootingState.LAUNCH;

			double currentTime = Timer.getFPGATimestamp();
			double elapsedTime = currentTime - timeAtPress;
			if (elapsedTime > 0.25) {
				shooterInState = ShootingState.LAUNCH;
			}
		} else if (currentOverallState == ShootingState.UNLAUNCH) {
			shooterOutState = ShootingState.UNLAUNCH;
			shooterInState = ShootingState.UNLAUNCH;
		}

		set(shooterInState, shooterOutState);
	}

	private void set(ShootingState shooterInState, ShootingState shooterOutState) {
		if (shooterInState == ShootingState.LAUNCH) {
			shooterIn.set(ControlMode.PercentOutput, shootInLaunchSpeed);
		} else if (shooterInState == ShootingState.UNLAUNCH) {
			shooterIn.set(ControlMode.PercentOutput, shootInUnlaunchSpeed);
		} else {
			shooterIn.set(ControlMode.PercentOutput, 0);
		}

		if (shooterOutState == ShootingState.LAUNCH) {
			shooterOut.set(shootOutLaunchSpeed);
		} else if (shooterOutState == ShootingState.UNLAUNCH) {
			shooterOut.set(shootOutUnlaunchSpeed);
		} else {
			shooterOut.set(0);
		}
	}
}
