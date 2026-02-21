// FRC Team 4034

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
	public static final class ConfigConsts {
		// Shortened names for convenience:
		// * lf: left-front
		// * rf: right-front
		// * lb: left-back
		// * rb: right-back

		public static final int lfSpeedMotorId = 3;
		public static final int lfDirectionMotorId = 4;
		public static final int lfEncoderId = 5;
		public static final boolean reverseLfSpeedMotor = true;
		public static final boolean reverseLfDirectionMotor = true;
		public static final boolean reverseLfEncoder = false;
		public static final boolean reverseLfSpeedEncoder = false;

		public static final int rfSpeedMotorId = 6;
		public static final int rfDirectionMotorId = 7;
		public static final int rfEncoderId = 8;
		public static final boolean reverseRfSpeedMotor = false;
		public static final boolean reverseRfDirectionMotor = true;
		public static final boolean reverseRfEncoder = false;
		public static final boolean reverseRfSpeedEncoder = true;

		public static final int lbSpeedMotorId = 0;
		public static final int lbDirectionMotorId = 1;
		public static final int lbEncoderId = 2;
		public static final boolean reverseLbSpeedMotor = true;
		public static final boolean reverseLbDirectionMotor = true;
		public static final boolean reverseLbEncoder = false;
		public static final boolean reverseLbSpeedEncoder = false;

		public static final int rbSpeedMotorId = 9;
		public static final int rbDirectionMotorId = 10;
		public static final int rbEncoderId = 11;
		public static final boolean reverseRbSpeedMotor = false;
		public static final boolean reverseRbDirectionMotor = true;
		public static final boolean reverseRbEncoder = false;
		public static final boolean reverseRbSpeedEncoder = true;
	}

	public static final class ModuleConsts {
		/******************* Constants for speed wheels *******************/
		public static final double wheelDiameterMeters = Units.inchesToMeters(4);

		// OLD_TODO: I just picked 6.82 arbitrarily from WCP's gear ratio docs. We
		// need to figure out what value actually belongs here. See:
		// docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options
		// public static final double wheelRotationPerSpeedMotorRotation = 1 / 6.82;

		public static final double wheelRotationPerSpeedMotorRotation = 1 / 6.2;

		// A.K.A., the circumference of a wheel.
		public static final double metersPerWheelRotation = Math.PI * wheelDiameterMeters;
		public static final double speedMotorRotationToMeters = wheelRotationPerSpeedMotorRotation
				* metersPerWheelRotation;

		// "P" component of our direction PID controller.
		public static final double directionP = 0.75 / Math.PI;
	}

	public static final class DriveConsts {
		// Distance between right and left wheels.
		public static final double trackWidth = Units.inchesToMeters(22);
		// Distance between front and back wheels.
		public static final double wheelBase = Units.inchesToMeters(22);
		public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
				new Translation2d(wheelBase / 2, -trackWidth / 2),
				new Translation2d(wheelBase / 2, trackWidth / 2),
				new Translation2d(-wheelBase / 2, -trackWidth / 2),
				new Translation2d(-wheelBase / 2, trackWidth / 2));

		// The factor we divide our desired "metersPerSec" by to actually set our
		// motor's speed.
		// TODO: This value's been copy+pasted from the reference project. Once we've
		// validated that our `getDrivePosition()` works, we can hopefully use
		// `getDriveVelocity()` to map our [-1, 1] motor inputs to the max velocity
		// we want the wheels to move at.
		public static final double maxMetersPerSecToMotorSpeed = 5;

		// TODO: This value's been copy+pasted from the reference project. Need to fine
		// tune this.
		public static final double maxRadPerSecToMotorSpeed = 2 * 2 * Math.PI;
	}
}
