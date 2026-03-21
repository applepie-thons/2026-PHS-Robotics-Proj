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

	private ShootingState currentOverallState = ShootingState.NOTHING;
	double timeAtPress = 0.0;

	public Shooter() {
		this.shooterIn = new TalonSRX( ConfigConsts.shooterInMotorId );
		this.shooterOut = new TalonFX( ConfigConsts.shooterOutKrakenMotorId );
	}

	public void setShootingState(ShootingState newOverallState) {
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

		set(shooterOutState, shooterInState);
	}

	private void set(ShootingState shooterInState, ShootingState shooterOutState) {
		// TODO: Double check that the directions of these inputs are correct.
		if (shooterInState == ShootingState.LAUNCH) {
			shooterIn.set(ControlMode.PercentOutput, -1);
		} else if (shooterInState == ShootingState.UNLAUNCH) {
			shooterIn.set(ControlMode.PercentOutput, 0.25);
		} else {
			shooterIn.set(ControlMode.PercentOutput, 0);
		}

		if (shooterOutState == ShootingState.LAUNCH) {
			shooterOut.set(-0.75);
		} else if (shooterOutState == ShootingState.UNLAUNCH) {
			shooterOut.set(0.75);
		} else {
			shooterOut.set(0);
		}
	}
}
