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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
	// for turn_to_degree() function
	private PIDController turn_pid = new PIDController(1.8, 0, 0);


	// --- For move meters function --- //
	private double initial_module_dist = 0.0;
	private PIDController move_pid = new PIDController(0.6, 0, 0);

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

	/*
	private Hysteresis2 xHysteresis = new Hysteresis2(
		1, DriveConsts.maxMetersPerSecToMotorSpeed / 1.8);
	private Hysteresis2 yHysteresis = new Hysteresis2(
		1, DriveConsts.maxMetersPerSecToMotorSpeed / 1.8);
	private Hysteresis2 rotHysteresis = new Hysteresis2(
		1, DriveConsts.maxRadPerSecToMotorSpeed / 1.8);
	*/

	public SwerveModulePosition[] getModulePositions() {
		// TODO: The ordering of the modules here is guessed based on how we need to pass
		// swerve module states in `setModules()`. Confirm that this is correct.
		SwerveModulePosition[] modulePositions = {
			rfModule.getModulePosition(),
			lfModule.getModulePosition(),
			rbModule.getModulePosition(),
			lbModule.getModulePosition()
		};
		return modulePositions;
	}

	public SwerveDrive() {
		// Calibrate the the NavXMXP in a separate thread, so that it doesn't block other initialization.
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				navxMxp.reset();

				// initialize after gyro is reset, because it needs the gyro position for initialization
				odometry = new SwerveDriveOdometry(
					DriveConsts.driveKinematics,
					navxMxp.getRotation2d(),
					getModulePositions());

			} catch (Exception e) {
			}
		}).start();
		turn_pid.enableContinuousInput(-Math.PI, Math.PI);
		turn_pid.setTolerance(0.035);

		move_pid.setTolerance(0.2);
	}

	public void setModules(double xSpeed, double ySpeed, double turnSpeed) {
		// TODO: See if these improve the feel of driving.
		// xSpeed = xHysteresis.nextValue(xSpeed);
		// ySpeed = yHysteresis.nextValue(ySpeed);
		// turnSpeed = rotHysteresis.nextValue(turnSpeed);

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

		odometry.update(navxMxp.getRotation2d(), getModulePositions());
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

		odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public void stopModules() {
		lfModule.stop();
		rfModule.stop();
		lbModule.stop();
		rbModule.stop();

		odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}



	public boolean turn_to_degree(double degree, double errorTolerance) {
		double current_degree = -navxMxp.getRotation2d().getRadians();

		// Subtract 180 to put the 0-360 degree input in the -180 to 180 range. This is to match the gyro's radian reading which spans from -pi to pi.
		double result = turn_pid.calculate(current_degree, deg_to_rad(degree - 180));
		SmartDashboard.putNumber("pid output", result);
		double error = turn_pid.getError();
		SmartDashboard.putNumber("pid error", error);
		if(!turn_pid.atSetpoint()){
			setModules(0.0, 0.0, result);
			return(false);
		}
		else{
			setModules(0.0, 0.0, 0.0);
			return(true);
		}
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

	// -------- Commands -------- //

	public class MoveToCmd {

		SwerveDrive swerveDrive;
		double start_dist = 0.0;
		double meters = 0.0;
		double Xdirection = 0.0;
		double Ydirection = 0.0;

		public MoveToCmd(SwerveDrive swerve_drive, double Meters, double X_direction, double Y_direction) {
			this.swerveDrive = swerve_drive;
			this.meters = Meters;
			this.Xdirection = X_direction;
			this.Ydirection = Y_direction;
		}


		public void commandInit() {
			start_dist = lfModule.getDrivePosition();
		}

		public boolean commandPeriodic() {

			double lf_dist = lfModule.getDrivePosition();

			double current_dist = lf_dist - start_dist;

			double pid_result = move_pid.calculate(current_dist, meters);

			// smartdashboard things
			SmartDashboard.putNumber("lf module distance", lf_dist);
			SmartDashboard.putNumber("move pid result", pid_result);
			SmartDashboard.putNumber("move pid error", move_pid.getError());

			if(!move_pid.atSetpoint()){
				setModules(pid_result * Xdirection, pid_result * Ydirection, 0.0);
				return(false);
			}
			else{
				setModules(0, 0, 0);
				return(true);
			}

		}

	}


	public class TurnToCmd {
		SwerveDrive swerve_drive;
		double target_degree;
		double error_tolerance;

		public TurnToCmd(SwerveDrive swerveDrive, double targetDegree, double errorTolerance){
			this.swerve_drive = swerveDrive;
			this.target_degree = targetDegree;
			this.error_tolerance = errorTolerance;
		}

		public boolean commandPeriodic() {
			return(turn_to_degree(target_degree, error_tolerance));
		}

	}
}
