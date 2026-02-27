// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.studica.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.DriveConsts;

// Unused imports
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.Servo;

public class Robot extends TimedRobot {
	// ------ Game Controllers ------ //
	private final XboxController controllerRed = new XboxController(0);
	private Intake intake = new Intake(new TalonFX(14), new TalonFX(15), new TalonFX(16));
	private TalonSRX shooterIntake = new TalonSRX(12);
	private TalonSRX shooterLaunch = new TalonSRX(13);
	private boolean shooterState = false;

	// ------ Debug variables for controlling swerve modules directly ------ //
	private int currModuleStrIndex = 0;
	private String moduleStrs[] = { "lf", "rf", "lb", "rb" };

	// encoder thingy
	/*
	private final Encoder testEncoder =
		new Encoder( 1, 0, false, Encoder.EncodingType.k2X );
	*/

	// ----- Swerve ----- //
	private SwerveDrive swerve_drive = new SwerveDrive();

	// ----- Deadzone + Hysteresis ----- //
	// hysteresis currently disabled because josh likes max speed better
	private double deadzone = 0.2;
	private double last_update_stickMagnitude = 0.0; // magnitude of the joystick on last update
	private double diff_threshold = 0.1; // how fast to pull the joystick to trigger hysteresis
	private double increase_speed = 5.5; // how fast the speed increases during hysteresis
	private double hysteresis_mult = 1.0; // the multiplier for the speed to make hysteresis work

	// ---- deltatime ----//
	private double last_update_timer = 0.0;
	private double deltaTime = 0.0;

	// ------ Gyro ------ //
	private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();
	private boolean is_auto_turning = false;
	private boolean first_auto_turn_call = true;
	private double auto_turn_direct = 1.0;

	// ------ Sonar ------ //
	public AnalogInput ultrasonicSensor = new AnalogInput(0);
	double voltageScaleFactor = 0;

	// ------ TalonFX test rotation thing ------ //
	TalonFX motorTestThing = new TalonFX(8);

	// ------ Linear Actuator ------ //
	// Servo linearActuator = new Servo(0);

	public Robot() {}

	@Override
	public void robotPeriodic() {
		voltageScaleFactor = 5/RobotController.getVoltage5V();
	}

	@Override
	public void robotInit() {
		// Setup for ADXR Gyro
		adxrGyro.reset();
		adxrGyro.calibrate();

		// testEncoder.reset();

		CameraServer.startAutomaticCapture(0);
	}

	@Override
	public void autonomousInit() {
		CameraServer.startAutomaticCapture();
	}

	@Override
	public void autonomousPeriodic() {
		// this was for the test encoder
		// encoderRotations = testEncoder.getDistance();
		// SmartDashboard.putNumber("test Encoder Value", encoderRotations);
	}

	@Override
	public void teleopInit() {}

	/** This function is called periodically during operator control. */

	public void teleopPeriodic() {
		// calculate deltaTime
		double currentTime = Timer.getFPGATimestamp();
		deltaTime = currentTime - last_update_timer;
		SmartDashboard.putNumber("deltaTime", deltaTime);


		SmartDashboard.putNumber("gyro rot degree", swerve_drive.navxMxp.getRotation2d().getDegrees());
		swerve_drive.turn_to_degree(0.0);

		// Divide by 5 to limit translate speed.
		double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
		double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);

		// Divide by 5 to limit rotation speed.
		double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 5);

		// swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);

		// teleopDriveTest();

		// keep at bottom so it only updates at the end and is usable in entire function
		last_update_timer = Timer.getFPGATimestamp();

		// double currentDistanceCentimeters = ultrasonicSensor.getValue() * 0.125;
		// double currentDistanceInches = ultrasonicSensor.getValue() * ultrasonicSensor.getVoltage() * 1.1;
		//double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;

		//SmartDashboard.putNumber("Sonar Distance (Inches)", currentDistanceInches);
		//SmartDashboard.putNumber("Voltage Scale Factor", voltage_scale_factor);

		// I fucking hate sonars so much bro jesus christ
		double sensorRange = ultrasonicSensor.getVoltage()*voltageScaleFactor;
		double sensorInches = sensorRange * 39.3442622951;
		double sensorCentimeters = sensorInches * 2.54;
		SmartDashboard.putNumber("Sensor Range", sensorRange);
		SmartDashboard.putNumber("Sensor Range (Inches)", sensorInches);
		SmartDashboard.putNumber("Sensor Range (Centimeters)", sensorCentimeters);

		SmartDashboard.putNumber("test motor encoder rotations", motorTestThing.getPosition().getValueAsDouble());
	}


	// Contains test code for controlling an individual swerve modu le. Actual code for
	// controlling swerve as a whole should probably be implemented in SwerveDrive.java.
	// This function specifically contains implementations for:
	//
	//    - Hysteresis
	//    - Joystick deadzone
	//
	private void teleopDriveTest() {

		double leftJoystickX = controllerRed.getLeftX();
		double leftJoystickY = controllerRed.getLeftY();
		double rightJoystick = controllerRed.getRightX();

		double joystickMagnitude = Math.sqrt(Math.pow(leftJoystickX, 2) + Math.pow(leftJoystickY, 2));

		/*
		// commented out because this deadzone will only work for swerve drive and cause
		// problems for tank
		if(deadzone > joystickMagnitude){
			throttle = 0.0;
		}
		*/

		// calculate joystick speed
		double joystick_diff = Math.abs(joystickMagnitude - last_update_stickMagnitude);
		SmartDashboard.putNumber("joystick_diff", joystick_diff);

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
	}

	private void calculate_hysteresis() {
		hysteresis_mult += increase_speed * deltaTime;
		SmartDashboard.putNumber("hysteresis multiplier", hysteresis_mult);
	}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {
		

		// Divide by 5 to limit translate speed.
		double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);
		double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 5);

		// Divide by 5 to limit rotation speed.
		double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 5);

		swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);




		if (controllerRed.getBButtonPressed()) {
			boolean currentIntakeState = intake.getIntakingState();
			intake.setIntakingState(!currentIntakeState);
		}
		intake.runIntake();


		if (controllerRed.getAButtonPressed()) {
			shooterState = !shooterState;
		}
		if (shooterState) {
			shooterIntake.set(ControlMode.PercentOutput, -1);
			shooterLaunch.set(ControlMode.PercentOutput, -1);
		}
		else {
			shooterIntake.set(ControlMode.PercentOutput, 0);
			shooterLaunch.set(ControlMode.PercentOutput, 0);
		}
		

		/*
		// Linear actuator test.
		double leftTriggerAxis = controllerRed.getLeftTriggerAxis();
		SmartDashboard.putNumber("Left trigger axis", leftTriggerAxis);
		linearActuator.set(leftTriggerAxis);
		*/
	}

	/*
	public double getAdxrGyro() {
		double adxrGyro_degrees = adxrGyro.getRotation2d().getDegrees();
		if (adxrGyro_degrees < 360.0 && adxrGyro_degrees > -360.0) {
			adxrGyro_degrees += 360.0;
		} else if (adxrGyro_degrees < -360.0) {
			adxrGyro_degrees += 360.0 * Math.abs(adxrGyro_degrees) / 360;
		}
		return Math.abs(adxrGyro_degrees) % 360.0;
	}

	// Old implementation for making the robot turn to particular angle using the
	// ADXR gyroscope.
	public void arcadeTurnToDegree() {
		double convertedGyro = getAdxrGyro();
		SmartDashboard.putNumber("gyroRotation", convertedGyro);

		double throttle = 0.25;
		double leftJoystick = controllerRed.getLeftY();
		double rightJoystick = controllerRed.getRightY();

		if(controllerRed.getAButtonPressed() && is_auto_turning == false){
			is_auto_turning = true;
			first_auto_turn_call = true;
		}

		if(is_auto_turning == false) {
			robotDrive.arcadeDrive(leftJoystick * throttle, -rightJoystick * throttle);
		}
		else {
			turn_to_degree_old(convertedGyro, 0.0, 0.5, 10.0);
		}

	}
	*/

	private void turn_to_degree_old(double current_degree, double target_degree, double speed, double accuracy) {
		double diff = (target_degree - current_degree) % 360;
		if (-accuracy < diff && diff < accuracy) {
			is_auto_turning = false;
			return;
		}
		if (first_auto_turn_call == true) {
			auto_turn_direct = diff / Math.abs(diff);
			if (diff < -180 || diff > 180) {
				auto_turn_direct *= -1.0;
			}
			first_auto_turn_call = false;
		}

		SmartDashboard.putNumber("diff", diff);
		SmartDashboard.putNumber("direction", auto_turn_direct);
		// robotDrive.arcadeDrive(0.0, auto_turn_direct * speed);
	}
}

/*
 * // Unused function headers.
 *
 * @Override
 * public void disabledInit() {}
 *
 * @Override
 * public void disabledPeriodic() {}
 *
 */


/*
 *
 * @Override
 * public void simulationInit() {}
 *
 * @Override
 * public void simulationPeriodic() {}
 */
