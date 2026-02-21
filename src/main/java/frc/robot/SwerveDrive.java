// FRC Team 4034

package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
	// for turn_to_degree() function
	private PIDController turn_pid = new PIDController(0.3, 0, 0);
	// turn_pid.enableContinuousInput(-Math.PI, Math.PI);

	// Shortened names for convenience:
	// * lf: left-front
	// * rf: right-front
	// * lb: left-back
	// * rb: right-back
	private final SwerveModule lfModule = new SwerveModule(
			"lf",
			ConfigConsts.lfSpeedMotorId,
			ConfigConsts.lfDirectionMotorId,
			ConfigConsts.lfEncoderId,
			ConfigConsts.reverseLfSpeedMotor,
			ConfigConsts.reverseLfDirectionMotor,
			ConfigConsts.reverseLfEncoder,
			ConfigConsts.reverseLfSpeedEncoder);

	private final SwerveModule rfModule = new SwerveModule(
			"rf",
			ConfigConsts.rfSpeedMotorId,
			ConfigConsts.rfDirectionMotorId,
			ConfigConsts.rfEncoderId,
			ConfigConsts.reverseRfSpeedMotor,
			ConfigConsts.reverseRfDirectionMotor,
			ConfigConsts.reverseRfEncoder,
			ConfigConsts.reverseRfSpeedEncoder);

	private final SwerveModule lbModule = new SwerveModule(
			"lb",
			ConfigConsts.lbSpeedMotorId,
			ConfigConsts.lbDirectionMotorId,
			ConfigConsts.lbEncoderId,
			ConfigConsts.reverseLbSpeedMotor,
			ConfigConsts.reverseLbDirectionMotor,
			ConfigConsts.reverseLbEncoder,
			ConfigConsts.reverseLbSpeedEncoder);

	private final SwerveModule rbModule = new SwerveModule(
			"rb",
			ConfigConsts.rbSpeedMotorId,
			ConfigConsts.rbDirectionMotorId,
			ConfigConsts.rbEncoderId,
			ConfigConsts.reverseRbSpeedMotor,
			ConfigConsts.reverseRbDirectionMotor,
			ConfigConsts.reverseRbEncoder,
			ConfigConsts.reverseRbSpeedEncoder);

	public AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

	public SwerveDrive() {
		// Calibrate the the NavXMXP in a separate thread, so that it doesn't block
		// other initialization.
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				navxMxp.reset();
			} catch (Exception e) {
			}
		}).start();
	}

	public void setModules(double xSpeed, double ySpeed, double turnSpeed) {
		// TODO: Check that "getRotation2d()" returns an angle in radians, and that
		// it is CCW positive.
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed, ySpeed, turnSpeed, navxMxp.getRotation2d());

		SwerveModuleState[] moduleStates = DriveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(
				moduleStates, DriveConsts.maxMetersPerSecToMotorSpeed);

		/*
		 * lfModule.setDesiredState(moduleStates[0]);
		 * rfModule.setDesiredState(moduleStates[1]);
		 * lbModule.setDesiredState(moduleStates[2]);
		 * rbModule.setDesiredState(moduleStates[3]);
		 */

		boolean ignoreLowSpeed = true;
		lfModule.setDesiredState(moduleStates[1], ignoreLowSpeed);
		rfModule.setDesiredState(moduleStates[0], ignoreLowSpeed);
		lbModule.setDesiredState(moduleStates[3], ignoreLowSpeed);
		rbModule.setDesiredState(moduleStates[2], ignoreLowSpeed);
	}

	public void setWheelsToAngle(double angleRadians) {
		double speed = 0;
		Rotation2d rot2d = new Rotation2d(angleRadians);
		SwerveModuleState swerveModuleState = new SwerveModuleState(speed, rot2d);

		boolean ignoreLowSpeed = false;

		lfModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		rfModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		lbModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		rbModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
	}

	public void stopModules() {
		lfModule.stop();
		rfModule.stop();
		lbModule.stop();
		rbModule.stop();
	}

	public void turn_to_degree(double degree) {
		double current_degree = navxMxp.getRotation2d().getDegrees();

		if(current_degree < 0.0) {
			current_degree += Math.floor(Math.abs(current_degree) / 360) + 1 * 360;
		}
		SmartDashboard.putNumber("current limited degree", current_degree);
		double degree_diff = degree - current_degree;
	}

	public double deg_to_rad(double deg) {
		double rad = deg * Math.PI / 180;
		return rad;
	}

	public double rad_to_deg(double rad) {
		double deg = rad * (180 / Math.PI);
		return deg;
	}

	/***********************************************************************************/
	/* Helper functions/variables for debugging
	/***********************************************************************************/
	public void setModuleDirect(String moduleId, double speedMotorInput, double directionMotorInput) {
		switch(moduleId) {
		case "lf":
			lfModule.setSpeedDirect(speedMotorInput);
			lfModule.setDirectionDirect(directionMotorInput);
			break;
		case "rf":
			rfModule.setSpeedDirect(speedMotorInput);
			rfModule.setDirectionDirect(directionMotorInput);
			break;
		case "lb":
			lbModule.setSpeedDirect(speedMotorInput);
			lbModule.setDirectionDirect(directionMotorInput);
			break;
		case "rb":
			rbModule.setSpeedDirect(speedMotorInput);
			rbModule.setDirectionDirect(directionMotorInput);
			break;
		}
	}

	public void log() {
		lfModule.log();
		rfModule.log();
		lbModule.log();
		rbModule.log();
	}
}
