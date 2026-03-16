// FRC Team 4034

package frc.robot;

import com.studica.frc.AHRS.NavXComType;

import com.studica.frc.AHRS;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
	// for turn_to_degree() function
	private PIDController turn_pid = new PIDController(1.8, 0, 0);

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

	public SwerveDriveOdometry odometry;

	public SwerveDrive() {
		// Calibrate the the NavXMXP in a separate thread, so that it doesn't block other initialization.
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				navxMxp.reset();

				// initialize after gyro is reset, because it needs the gyro position for initialization
				//SwerveModulePosition[] module_positions = new SwerveModulePosition[] {new SwerveModulePosition(lfModule.getDrivePosition(), )}
				//odometry = new SwerveDriveOdometry(new SwerveDriveKinematics(null), navxMxp.getRotation2d(), module_positions);

			} catch (Exception e) {
			}
		}).start();
		turn_pid.enableContinuousInput(-Math.PI, Math.PI);
		turn_pid.setTolerance(0.035);
	}

	public void setModules(double xSpeed, double ySpeed, double turnSpeed) {
		// TODO: Check that "getRotation2d()" returns an angle in radians, and that it is CCW positive.
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed, ySpeed, turnSpeed, navxMxp.getRotation2d());

		SwerveModuleState[] moduleStates = DriveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(
				moduleStates, DriveConsts.maxMetersPerSecToMotorSpeed);

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

	public void turn_to_degree(double degree, double errorTolerance) {
		double current_degree = -navxMxp.getRotation2d().getRadians();

		// Subtract 180 to put the 0-360 degree input in the -180 to 180 range. This is to match the gyro's radian reading which spans from -pi to pi.
		double result = turn_pid.calculate(current_degree, deg_to_rad(degree - 180));
		SmartDashboard.putNumber("pid output", result);
		double error = turn_pid.getError();
		if(!turn_pid.atSetpoint()){
			setModules(0.0, 0.0, result);
		}
		else{
			setModules(0.0, 0.0, 0.0);
		}
		SmartDashboard.putNumber("pid error", error);
	}


	// ------- radian functions -------- //
	public double deg_to_rad(double deg){
		return deg * (Math.PI/180);
	}

	// ------- Helper functions/variables for debugging ------- //
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
