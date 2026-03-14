// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Intake.IntakePosition;

public class Robot extends TimedRobot {
	// ------ Game Controllers ------ //
	private final XboxController controllerRed = new XboxController(0);
	private final XboxController controllerYellow = new XboxController(1);

	private TalonSRX shooterIntake = new TalonSRX( ConfigConsts.shooterInMotorId );
	private TalonSRX shooterLaunch = new TalonSRX( ConfigConsts.shooterOutMotorId );

	private Intake intake = new Intake();

	private TalonFX climb = new TalonFX( ConfigConsts.climbMotorId );

	// ----- Swerve ----- //
	private SwerveDrive swerve_drive = new SwerveDrive();

	// ------ Debug variables for controlling swerve modules directly ------ //
	//private int currModuleStrIndex = 0;
	//private String moduleStrs[] = { "lf", "rf", "lb", "rb" };

	// ----- Deadzone + Hysteresis ----- //
	// hysteresis currently disabled because josh likes max speed better
	//private double deadzone = 0.2;
	private double last_update_stickMagnitude = 0.0; // magnitude of the joystick on last update
	private double diff_threshold = 0.1; // how fast to pull the joystick to trigger hysteresis
	private double increase_speed = 5.5; // how fast the speed increases during hysteresis
	private double hysteresis_mult = 1.0; // the multiplier for the speed to make hysteresis work

	// ---- turn to degree ---- //
	private boolean is_auto_turning = false;

	// ---- deltatime ----//
	private double last_update_timer = 0.0;
	private double deltaTime = 0.0;

	// ------ Gyro ------ //
	private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();

	/* // old turn to degree function //
	private boolean is_auto_turning = false;
	private boolean first_auto_turn_call = true;
	private double auto_turn_direct = 1.0;
	*/

	// ------ Sonar ------ //
	public AnalogInput ultrasonicSensor = new AnalogInput(0);
	double voltageScaleFactor = 0;

	// ------ Camera ------ //
	private final UsbCamera camera;

	public Robot() {
		// TODO (Sahil): Check that is actually drops the camera's resolution.
		this.camera = CameraServer.startAutomaticCapture();
        this.camera.setPixelFormat(PixelFormat.kMJPEG);
        this.camera.setResolution(320, 240);
        this.camera.setFPS(30);
	}

	@Override
	public void robotPeriodic() {
		voltageScaleFactor = 5/RobotController.getVoltage5V();
	}

	@Override
	public void robotInit() {
		// Setup for ADXR Gyro
		adxrGyro.reset();
		adxrGyro.calibrate();
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	public void redControllerPeriodic() {
		// ------------------------------- Swerve ------------------------------- //
		double turn_tolerance = 0.2;
		if (controllerRed.getYButton()) {
			swerve_drive.turn_to_degree(0, turn_tolerance);
		} else if (controllerRed.getXButton()) {
			swerve_drive.turn_to_degree(180, turn_tolerance);
		} else {
			// Divide by 5 to limit translate speed.
			double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
			double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
			// Divide by 5 to limit rotation speed.
			double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 5);

			swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
		}

		// ------------------------------- Climb ------------------------------- //
		double climbSpeed = controllerRed.getLeftTriggerAxis();
		climbSpeed *= climbSpeed;
		double climbBackSpeed = controllerRed.getRightTriggerAxis();
		climbBackSpeed *= climbBackSpeed * -1;
		// Try setting voltage directly?
		climb.set(climbSpeed + climbBackSpeed);
	}

	public void yellowControllerPeriodic() {
		// ------------------------------- Intake ------------------------------- //
		// Set desired speed of the intaking wheels.
		if (controllerYellow.getBButton()) {
			intake.setIntakeSpeed(-0.55);
		}
		else if (controllerYellow.getYButton()) {
			intake.setIntakeSpeed(0.55);
		}
		else {
			intake.setIntakeSpeed(0);
		}

		// Set desired position of the intake arm when in `intake.getAutoMode()`
		// is true.
		if (controllerYellow.getStartButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.OUT);
		}
		else if(controllerYellow.getBackButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.IN);;
		}

		// Toggle between the intake arm control modes. There are two different
		// control modes, tracked by the boolean return by `intake.getAutoMode()`.
		//
		//     - autoMode=True: Using PID, the arm will move to the position specified
		//                      by `intake.manualSetIntakePosition()`.
		//     - autoMode=False: The arm is controlled directly via an analog user input.
		if (controllerYellow.getRightBumperButtonPressed()) {
			intake.swapPivotMode();
		}

		// Actually set the state of the intake wheels and arm motors based on what was
		// configured above.
		if (intake.getAutoMode()) {
			intake.intakePeriodic();
		}
		else {
			if (controllerYellow.getRightTriggerAxis() > 0) {
				intake.intakePeriodic(-1 * controllerYellow.getRightTriggerAxis());
			}
			else if (controllerYellow.getLeftTriggerAxis() > 0) {
				intake.intakePeriodic(controllerYellow.getLeftTriggerAxis());
			}
			else {
				intake.intakePeriodic(0);
			}
		}

		// ------------------------------- Shooter ------------------------------- //
		if (controllerYellow.getAButton()) {
			shooterIntake.set(ControlMode.PercentOutput, -1);
			shooterLaunch.set(ControlMode.PercentOutput, -1);
		}
		else if (controllerYellow.getXButton()) {
			shooterIntake.set(ControlMode.PercentOutput, 1);
			shooterLaunch.set(ControlMode.PercentOutput, 1);
		}
		else {
			shooterIntake.set(ControlMode.PercentOutput, 0);
			shooterLaunch.set(ControlMode.PercentOutput, 0);
		}
	}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
		redControllerPeriodic();
		yellowControllerPeriodic();
	}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {
		// calculate deltaTime
		double currentTime = Timer.getFPGATimestamp();
		deltaTime = currentTime - last_update_timer;
		SmartDashboard.putNumber("delta_time", deltaTime);
		SmartDashboard.putNumber("gyro_degrees", swerve_drive.navxMxp.getRotation2d().getDegrees());

		// Divide by 5 to limit speed.
		double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
		double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
		double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 5);

		if(!is_auto_turning){
			swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
		}

		 //turn_to_degree forward + backward
		SmartDashboard.putNumber("gyro radians", swerve_drive.navxMxp.getRotation2d().getRadians());

		if (controllerRed.getYButton()) {
			is_auto_turning = true;
			swerve_drive.turn_to_degree(0, 0.2);
		}
		else if (controllerRed.getXButton()) {
			is_auto_turning = true;
			swerve_drive.turn_to_degree(180, 0.2);
		}
		else is_auto_turning = false;

		// ------------ old hysteresis + deadzone that we did a while ago with swerve ---------- //

		// --------not active currently dont expect it to do anything

		double joystickMagnitude = Math.sqrt(Math.pow(controllerRed.getLeftX(), 2) + Math.pow(controllerRed.getLeftY(), 2));

		/*
		// commented out because this deadzone will only work for swerve drive and cause
		// problems for tank
		if(deadzone > joystickMagnitude){
			throttle = 0.0;
		}
		*/

		// calculate joystick speed
		double joystick_diff = Math.abs(joystickMagnitude - last_update_stickMagnitude);


		if (joystick_diff > diff_threshold) {
			hysteresis_mult = 0.0;
		}

		if (hysteresis_mult < 1.0) {
			calculate_hysteresis();
		} else {
			hysteresis_mult = 1.0;
		}

		// keep at bottom so it only updates at the end and is usable in entire function
		last_update_stickMagnitude = joystickMagnitude;
		last_update_timer = Timer.getFPGATimestamp();
	}

	private void calculate_hysteresis() {
		hysteresis_mult += increase_speed * deltaTime;
		SmartDashboard.putNumber("hysteresis multiplier", hysteresis_mult);
	}

	@Override
  	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		intake.logIntake();
	}

	public void sonarTestPeriodic() {
		// double currentDistanceCentimeters = ultrasonicSensor.getValue() * 0.125;
		// double currentDistanceInches = ultrasonicSensor.getValue() * ultrasonicSensor.getVoltage() * 1.1;
		// double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;

		// SmartDashboard.putNumber("Sonar Distance (Inches)", currentDistanceInches);
		// SmartDashboard.putNumber("Voltage Scale Factor", voltage_scale_factor);

		// "I hate sonars so much bro jesus christ"
		//                					- Michael Milward, 2026
		double sensorRange = ultrasonicSensor.getVoltage()*voltageScaleFactor;
		double sensorInches = sensorRange * 39.3442622951;
		double sensorCentimeters = sensorInches * 2.54;
		SmartDashboard.putNumber("Sensor Range", sensorRange);
		SmartDashboard.putNumber("Sensor Range (Inches)", sensorInches);
		SmartDashboard.putNumber("Sensor Range (Centimeters)", sensorCentimeters);
	}
}

/*
 * // Unused function headers.
 *
 *
 *
 * @Override
 * public void simulationInit() {}
 *
 * @Override
 * public void simulationPeriodic() {}
 */
