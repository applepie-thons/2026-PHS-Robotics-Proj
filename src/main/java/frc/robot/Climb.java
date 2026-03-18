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

		this.minClimbPosition = 0;
		this.maxClimbPosition = 69;
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

class ClimbExtendCmd extends CommandBase {
	Climb climb;
	// This constructor can be used to customize how we want this command to behave.
	// A "move-to-distance" command for swerve could accept a distance and direction
	// in its constructor, for example.
	//
	// The constructor for any command must also take in the object it wants to control
	// (`Climb` in this case, `SwerveDrive` in the case of swerve auto), so that it can
	// actually control it in `commandPeriodic()`.
	public ClimbExtendCmd(Climb inClimb) {
		this.climb = inClimb;
	}

	boolean commandPeriodic() {
		double retractInput = 0;
		double extendInput = 0.75;
		boolean ignoreLimits = false;
		return climb.set(retractInput, extendInput, ignoreLimits);
	}
}

class ClimbRetractCmd extends CommandBase {
	Climb climb;
	public ClimbRetractCmd(Climb inClimb) {
		this.climb = inClimb;
	}

	boolean commandPeriodic() {
		// TODO: Maybe apply hysteresis here. Retracting with full force right away is a little
		// scary, but we might need to eventually apply full force to lift the bot up.
		double retractInput = 1;
		double extendInput = 0;
		boolean ignoreLimits = false;
		return climb.set(retractInput, extendInput, ignoreLimits);
	}
}
