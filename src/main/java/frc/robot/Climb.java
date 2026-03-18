// FRC Team 4034

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConfigConsts;

public class Climb {

	private TalonFX climbMotor;

	private double minClimbPosition;
	private double maxClimbPosition;

	public Climb() {
		this.climbMotor = new TalonFX( ConfigConsts.climbMotorId );
		// Reset the climb motor encoder's position. Assume that it starts fully retracted.
		this.climbMotor.setPosition(0);
		// `Brake` means that when we feed a value of `0` to the motor it will stop
		// immediately (i.e., brake), rather than keeping momentum from previous movement.
		// This important for preventing the rope from unwinding when extending.
		this.climbMotor.setNeutralMode(NeutralModeValue.Brake);
	}

	public boolean set(double retractInput, double extendInput, boolean ignoreLimits) {
		double climbSpeed = retractInput + extendInput;
		if (!ignoreLimits) {
			double currClimbPosition = climbMotor.getPosition().getValueAsDouble();

			if (climbSpeed < 0 && currClimbPosition <= minClimbPosition) {
				climbSpeed = 0;
			} else if (climbSpeed > 0 && currClimbPosition >= maxClimbPosition) {
				climbSpeed = 0;
			}
		}

		climbMotor.set(climbSpeed);

		boolean finishedClimbing = ( climbSpeed == 0 );
		return finishedClimbing;
	}

	public void resetEncoder() {
		// Reset the climb motor encoder's position on a button press for debugging purposes.
		climbMotor.setPosition(0);
	}

	public void log() {
		SmartDashboard.putNumber("currClimbPosition", climbMotor.getPosition().getValueAsDouble());
	}
}
