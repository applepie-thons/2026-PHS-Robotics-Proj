// FRC Team 4034

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
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
		this.maxClimbPosition = 71.3;
	}

	public double getPosition() {
		int reverse = ConfigConsts.reverseClimbMotor ? -1 : 1;
		return climbMotor.getPosition().getValueAsDouble() * reverse;
	}

	public boolean set(double retractInput, double extendInput, boolean ignoreLimits) {
		double climbSpeed = (retractInput * -1) + extendInput;
		if (!ignoreLimits) {
			double currClimbPosition = getPosition();

			if (climbSpeed < 0 && currClimbPosition <= minClimbPosition) {
				climbSpeed = 0;
			} else if (climbSpeed > 0 && currClimbPosition >= maxClimbPosition) {
				climbSpeed = 0;
			}
		}

		int reverse = ConfigConsts.reverseClimbMotor ? -1 : 1;
		climbMotor.set(climbSpeed * reverse);
		log();

		boolean finishedClimbing = ( climbSpeed == 0 );
		return finishedClimbing;
	}

	public void resetEncoder() {
		// Reset the climb motor encoder's position on a button press for debugging purposes.
		climbMotor.setPosition(0);
	}

	public void log() {
		SmartDashboard.putNumber("currClimbPosition", getPosition());
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

	private double timeToMaxForHysteresis = 0.5;
	private double maxMotorValForHysteresis = 1;
	private Hysteresis2 hysteresis =
		new Hysteresis2(timeToMaxForHysteresis, maxMotorValForHysteresis);

	public ClimbExtendCmd(Climb inClimb) {
		this.climb = inClimb;
	}

	public boolean commandPeriodic() {
		double retractInput = 0;
		double extendInput = hysteresis.nextValue(0.75);
		boolean ignoreLimits = false;
		return climb.set(retractInput, extendInput, ignoreLimits);
	}
}

class ClimbRetractCmd extends CommandBase {
	Climb climb;
	double startTime;

	private double timeToMaxForHysteresis = 0.5;
	private double maxMotorValForHysteresis = 1;
	private Hysteresis2 hysteresis =
		new Hysteresis2(timeToMaxForHysteresis, maxMotorValForHysteresis);

	public ClimbRetractCmd(Climb inClimb) {
		this.climb = inClimb;
	}

	public void commandInit() {
		this.startTime = Timer.getFPGATimestamp();
	}

	public boolean commandPeriodic() {
		double elapsedTime = Timer.getFPGATimestamp() - startTime;
		if (elapsedTime > 5) {
			// Stop attempting to climb after 5 seconds to avoid stressing the motor too much.
			// The 1-to-1 brake gearbox prevents us from slipping down anyways.
			return true;
		}

		double retractInput = hysteresis.nextValue(1);
		double extendInput = 0;

		boolean ignoreLimits = false;
		return climb.set(retractInput, extendInput, ignoreLimits);
	}
}
