// FRC Team 4034

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ConfigConsts;

public class Shooter {
	public enum ShootingState {
		NOTHING,
		LAUNCH,
		UNLAUNCH;
	}

	private TalonSRX shooterIn;
	private TalonFX shooterOut;

	public Shooter() {
		this.shooterIn = new TalonSRX( ConfigConsts.shooterInMotorId );
		this.shooterOut = new TalonFX( ConfigConsts.shooterOutKrakenMotorId );
	}

	public void set(ShootingState shooterInState, ShootingState shooterOutState) {
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
