// FRC Team 4034

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.ModuleConsts;

public class SwerveModule {
	private final TalonFX speedMotor;
	private final TalonFX directionMotor;
	// Tracks the current angle of motor. This retains its values over a powercycle,
	// unlike the TalonFX's builtin encoder.
	private final CoreCANcoder absoluteEncoder;

	private final boolean reverseSpeedMotor;
	private final boolean reverseDirectionMotor;
	private final boolean reverseAbsoluteEncoder;
	private final boolean reverseSpeedEncoder;

	private final PIDController directionController;

	// Used to tell modules apart in SmartDashboard logging.
	String moduleName;

	public SwerveModule(String moduleName, int speedMotorId, int directionMotorId,
			int absoluteEncoderId, boolean reverseSpeedMotor,
			boolean reverseDirectionMotor, boolean reverseAbsoluteEncoder,
			boolean reverseSpeedEncoder) {
		this.speedMotor = new TalonFX(speedMotorId);
		this.directionMotor = new TalonFX(directionMotorId);
		this.absoluteEncoder = new CoreCANcoder(absoluteEncoderId);

		this.reverseSpeedMotor = reverseSpeedMotor;
		this.reverseDirectionMotor = reverseDirectionMotor;
		this.reverseAbsoluteEncoder = reverseAbsoluteEncoder;
		this.reverseSpeedEncoder = reverseSpeedEncoder;

		this.directionController = new PIDController(ModuleConsts.directionP, 0, 0);
		this.directionController.enableContinuousInput(-Math.PI, Math.PI);

		this.moduleName = moduleName;

		// Reset the speedMotor's encoder.
		this.speedMotor.setPosition(0);
	}

	public double getDrivePosition() {
		double motorRotations = this.speedMotor.getPosition().getValue().baseUnitMagnitude();

		// Do the following unit conversion:
		// (MotorRotation) * (Meters / MotorRotation) -> (Meters)
		double meters = motorRotations * ModuleConsts.speedMotorRotationToMeters;

		int reverseFactor = (this.reverseSpeedEncoder ? -1 : 1);
		return meters * reverseFactor;
	}

	public double getDriveVelocity() {
		double motorRotationsPerSec = this.speedMotor.getVelocity().getValue().baseUnitMagnitude();

		// Do the following unit conversion:
		// (MotorRotation / Sec) * (Meters / MotorRotation) -> (Meters / Sec)
		double metersPerSec = motorRotationsPerSec * ModuleConsts.speedMotorRotationToMeters;

		int reverseFactor = (this.reverseSpeedEncoder ? -1 : 1);
		return metersPerSec * reverseFactor;
	}

	public double getAbsoluteEncoderRad() {
		double angleRad = this.absoluteEncoder.getAbsolutePosition().getValue().baseUnitMagnitude();
		return (this.reverseAbsoluteEncoder ? -angleRad : angleRad);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
				getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
	}

	public void setDesiredState(SwerveModuleState state, boolean ignoreLowSpeed) {
		// This "if" condition prevents the wheels from swinging back to their
		// neutral position when the joysticks are let go.
		// TODO: The "0.001" may need to be increased depending on the behavior we
		// observe when testing this.
		if (ignoreLowSpeed && Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		state.optimize(getState().angle);

		SmartDashboard.putNumber("DesiredAngle", state.angle.getDegrees());

		double newSpeedMotorVal = state.speedMetersPerSecond / DriveConsts.maxMetersPerSecToMotorSpeed;
		int speedReverse = reverseSpeedMotor ? -1 : 1;
		speedMotor.set(newSpeedMotorVal * speedReverse);

		double newDirectionMotorVal = directionController.calculate(
				getAbsoluteEncoderRad(), state.angle.getRadians());
		SmartDashboard.putNumber("DirMotorVal", newDirectionMotorVal);
		int directionReverse = reverseDirectionMotor ? -1 : 1;
		directionMotor.set(newDirectionMotorVal * directionReverse);
	}

	public void stop() {
		speedMotor.set(0);
		directionMotor.set(0);
	}

	/***********************************************************************************/
	/* Helper functions/variables for debugging */
	/***********************************************************************************/
	public void setSpeedDirect(double speed) {
		int reverse = reverseSpeedMotor ? -1 : 1;
		speedMotor.set(speed * reverse);
	}

	public void setDirectionDirect(double directionMagnitude) {
		int reverse = reverseDirectionMotor ? -1 : 1;
		directionMotor.set(directionMagnitude * reverse);
	}

	public void log() {
		SmartDashboard.putNumber(moduleName + " drivePosition",
				getDrivePosition());
		SmartDashboard.putNumber(moduleName + " driveVelocity",
				getDriveVelocity());
		SmartDashboard.putNumber(moduleName + " absEncoderDegrees",
				getAbsoluteEncoderRad() * (180 / Math.PI));
	}

}
