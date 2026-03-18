// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match; (might need later)
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;
import frc.robot.Intake.IntakePosition;

public class Robot extends TimedRobot {
	enum SwerveSpeedMode {
		SLOW,
		DEFAULT,
		FAST,
		EXTRA_FAST;
	}

	// ------ Game Controllers ------ //
	private final XboxController controllerRed = new XboxController(0);
	private final XboxController controllerYellow = new XboxController(1);

	private TalonSRX shooterIntake = new TalonSRX( ConfigConsts.shooterInMotorId );
	private TalonFX shooterLaunch = new TalonFX(27);
	private Intake intake = new Intake();
	private Climb climb = new Climb();

	private SwerveSpeedMode swerveSpeedMode = SwerveSpeedMode.DEFAULT;

	// ----- Swerve ----- //
	private SwerveDrive swerve_drive = new SwerveDrive();

	// ------ Debug variables for controlling swerve modules directly ------ //
	//private int currModuleStrIndex = 0;
	//private String moduleStrs[] = { "lf", "rf", "lb", "rb" };

	// ----- Deadzone + Hysteresis (Disabled) ----- //
	// hysteresis currently disabled because josh likes max speed better
	// to enable, multiply motor output by hysteresis_mult
	//private double deadzone = 0.2;
	private double last_update_stickMagnitude = 0.0; // magnitude of the joystick on last update
	private double diff_threshold = 0.1; // how fast to pull the joystick to trigger hysteresis
	private double increase_speed = 5.5; // how fast the speed increases during hysteresis
	private double hysteresis_mult = 1.0; // the multiplier for the speed to make hysteresis work

	// ---- Turn to Degree ---- //
	private boolean is_auto_turning = false;

	// ---- deltaTime ----//
	private double last_update_timer = 0.0;
	private double deltaTime = 0.0;
	private double auto_start_time = 0;

	// ------ Gyro ------ //
	private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();

	// ------ Sonar ------ //
	public AnalogInput ultrasonicSensor = new AnalogInput(0);
	double voltageScaleFactor = 0;

	// ------ Camera ------ //
	private final UsbCamera camera;

	// Linear Actuator
	Servo linearAcutator = new Servo(0);
	private double servoLastUpdateTime = 0;

	// Autonomous command sequence.
	ArrayList<CommandBase> autoTestCmds;
	private int currCmdIndex = 0;

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
		servoLastUpdateTime = Timer.getFPGATimestamp();
	}

	public void servoPeriodic() {
		double currTime = Timer.getFPGATimestamp();
		double servoElapsedTime = currTime - servoLastUpdateTime;

		double currValue = linearAcutator.get();
		double newValue;

		double servoIn = 0.2;
		double servoOut = 0.38;

		if (currValue <= servoIn && servoElapsedTime > 2.6) {
			newValue = servoOut;
			servoLastUpdateTime = currTime;
		} else if (currValue >= servoOut && servoElapsedTime > 2.6) {
			newValue = servoIn;
			servoLastUpdateTime = currTime;
		} else {
			newValue = currValue;
		}
		linearAcutator.set(newValue);
	}

	@Override
	public void autonomousInit() {
		auto_start_time = Timer.getFPGATimestamp();
		linearAcutator.set(0);
	}

	@Override
	public void autonomousPeriodic() {
		double currentTime = Timer.getFPGATimestamp();
		double elapsedTime = currentTime - auto_start_time;

		if (elapsedTime < 0.25) {
			shooterLaunch.set(-0.75);
		}
		else {
			shooterIntake.set(ControlMode.PercentOutput, -1);
			shooterLaunch.set(-0.75);
		}

		servoPeriodic();

		// --- a way to use the move meters function in auto (since its kind of confusing and bad) --------------------- //
		/*
		int current_auto_step = 1;
		boolean first_call = true;
		// ^^ both go in constructor ^^
		if(current_auto_step == 1){
			if(swerve_drive.move_meters_in_direction(2, 1, 0, first_call)){
				current_auto_step += 1;
				first_call = true;
			}else{
				first_call = false;
			}

		}
		else if(current_auto_step == 2){
			if(swerve_drive.move_meters_in_direction(2, -1, 0, first_call)){
				current_auto_step += 1;
				first_call = true;
			}else{
				first_call = false;
			}
		}

		// with this, the robot will (hopefully) move forward 2 meters then back 2 meters
		*/

	}

	public void redControllerPeriodic() {
		// ------------------------------- Swerve ------------------------------- //
		/*
		double turn_tolerance = 0.2;
		if (controllerRed.getYButton()) {
			swerve_drive.turn_to_degree(0, turn_tolerance);
		} else if (controllerRed.getXButton()) {
			swerve_drive.turn_to_degree(180, turn_tolerance);
		*/

		if (controllerRed.getXButtonPressed()) {
			swerveSpeedMode = SwerveSpeedMode.SLOW;
		} else if (controllerRed.getAButtonPressed()) {
			swerveSpeedMode = SwerveSpeedMode.DEFAULT;
		} else if (controllerRed.getBButtonPressed()) {
			swerveSpeedMode = SwerveSpeedMode.FAST;
		} else if (controllerRed.getYButtonPressed()) {
			swerveSpeedMode = SwerveSpeedMode.EXTRA_FAST;
		}

		double speedThrottle;
		if (swerveSpeedMode == SwerveSpeedMode.SLOW) {
			speedThrottle = 7.5;
		} else if (swerveSpeedMode == SwerveSpeedMode.DEFAULT) {
			speedThrottle = 4.2;
		} else if (swerveSpeedMode == SwerveSpeedMode.FAST) {
			speedThrottle = 2.5;
		} else if (swerveSpeedMode == SwerveSpeedMode.EXTRA_FAST) {
			speedThrottle = 1.8;
		} else {
			speedThrottle = 5;
		}

		// Divide by 5 to limit translate speed.
		double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / speedThrottle);
		double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / speedThrottle);
		// Divide by 5 to limit rotation speed.
		double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / speedThrottle);
		if(!is_auto_turning){
			swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
		}

		// ------------------------------- Climb ------------------------------- //
		// Square the inputs for finer-grain control.
		// TODO: I think something was rewired during competition, so the names 'retract'
		// and 'extend' have opposite meanings. Double check that.
		double climbRetractSpeed = Math.pow(controllerRed.getRightTriggerAxis(), 2);
		double climbExtendSpeed = Math.pow(controllerRed.getLeftTriggerAxis(), 2) * -1;
		boolean ignoreClimbLimit = controllerRed.getBackButton();
		climb.set(climbRetractSpeed, climbExtendSpeed, ignoreClimbLimit );
		if (controllerRed.getStartButtonPressed()) {
			climb.resetEncoder();
		}

		// old button-related turn_to_degree
		/*
		if (controllerRed.getLeftBumperButton()) {
			is_auto_turning = true;
			swerve_drive.turn_to_degree(0, 0.2);
		}
		else if (controllerRed.getRightBumperButton()) {
			is_auto_turning = true;
			swerve_drive.turn_to_degree(180, 0.2);
		}
		else{
			is_auto_turning = false;
		}
		*/
		// should make robot turn to whatever angle is being pressed on the d-pad
		if(!(controllerRed.getPOV() == -1)){
			is_auto_turning = true;
			swerve_drive.turn_to_degree(controllerRed.getPOV() - 180, 0.2);
		}
		else{
			is_auto_turning = false;
		}

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

		// Set desired position of the intake arm when in `intake.getAutoMode()` is true.
		if (controllerYellow.getStartButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.OUT);
		}
		else if(controllerYellow.getBackButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.IN);;
		}

		/*	Toggle between the intake arm control modes. There are two different control modes, tracked by the boolean return by `intake.getAutoMode()`.

			- autoMode=True: Using PID, the arm will move to the position specified by `intake.manualSetIntakePosition()`.
			- autoMode=False: The arm is controlled directly via an analog user input.
		*/
		if (controllerYellow.getRightBumperButtonPressed()) {
			intake.swapPivotMode();
		}

		// Actually set the state of the intake wheels and arm motors based on what was configured above.
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
		}
		else if (controllerYellow.getXButton()) {
			shooterIntake.set(ControlMode.PercentOutput, 0.25);
		}
		else {
			shooterIntake.set(ControlMode.PercentOutput, 0);

		}

		if (controllerYellow.getLeftBumperButton()) {
			// Shoot
			shooterLaunch.set(-0.75);
		} else if (controllerYellow.getXButton()) {
			// Unshoot
			shooterLaunch.set(0.75);
		} else {
			shooterLaunch.set(0);
		}
	}

	@Override
	public void teleopInit() {
		linearAcutator.set(0);
	}

	@Override
	public void teleopPeriodic() {
		servoPeriodic();
		redControllerPeriodic();
		yellowControllerPeriodic();
	}

	@Override
	public void testInit() {
		autoTestCmds.add( new ClimbExtendCmd( climb ) );
		autoTestCmds.add( new ClimbRetractCmd( climb ) );

		CommandBase firstCmd = autoTestCmds.get( 0 );
		firstCmd.commandInit();
	}

	public void testPeriodic2() {
		if(currCmdIndex == autoTestCmds.size()) {
			// We have iterated through all of the commands. There's nothing left to do,
			// so return early.
			return;
		}
		CommandBase currCmd = autoTestCmds.get( currCmdIndex );
		boolean cmdFinished = currCmd.commandPeriodic();

		if (cmdFinished) {
			currCmdIndex += 1;
			CommandBase nextCmd = autoTestCmds.get( currCmdIndex );
			nextCmd.commandInit();
		}
	}

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
			is_auto_turning = !swerve_drive.turn_to_degree(0, 0.2);
		}
		else if (controllerRed.getXButton()) {
			is_auto_turning = !swerve_drive.turn_to_degree(180, 0.2);
		}
		else is_auto_turning = false;

		// ------------ old hysteresis + deadzone that we did a while ago with swerve ---------- //
		// ---------- not active currently dont expect it to do anything ---------- //

		double joystickMagnitude = Math.sqrt(Math.pow(controllerRed.getLeftX(), 2) + Math.pow(controllerRed.getLeftY(), 2));

		/*
		// commented out because this deadzone will only work for swerve drive and cause problems for tank
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
		/*
		// ---------- Sonar (Unused) ---------- //
		- Yes, I still hate sonar. (Michael Milward, 2026)

		double currentDistanceCentimeters = ultrasonicSensor.getValue() * 0.125;
		double currentDistanceInches = ultrasonicSensor.getValue() * ultrasonicSensor.getVoltage() * 1.1;
		double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;

		SmartDashboard.putNumber("Sonar Distance (Inches)", currentDistanceInches);
		SmartDashboard.putNumber("Voltage Scale Factor", voltage_scale_factor);

		double sensorRange = ultrasonicSensor.getVoltage()*voltageScaleFactor;
		double sensorInches = sensorRange * 39.3442622951;
		double sensorCentimeters = sensorInches * 2.54;
		SmartDashboard.putNumber("Sensor Range", sensorRange);
		SmartDashboard.putNumber("Sensor Range (Inches)", sensorInches);
		SmartDashboard.putNumber("Sensor Range (Centimeters)", sensorCentimeters);
		*/
	}
}

/*
 * @Override
 * public void simulationInit() {}
 *
 * @Override
 * public void simulationPeriodic() {}
 */
