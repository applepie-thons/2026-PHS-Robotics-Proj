// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

//import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match; (might need later)
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
import frc.robot.Constants.DriveConsts;
import frc.robot.Intake.IntakePosition;
import frc.robot.Shooter.ShootingState;

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

	private Shooter shooter = new Shooter();
	private Intake intake = new Intake();
	private Climb climb = new Climb();

	private SwerveSpeedMode swerveSpeedMode = SwerveSpeedMode.DEFAULT;

	// ----- Swerve ----- //
	private SwerveDrive swerve_drive = new SwerveDrive();

	// ------ Debug variables for controlling swerve modules directly ------ //
	//private int currModuleStrIndex = 0;
	//private String moduleStrs[] = { "lf", "rf", "lb", "rb" };

	// ----- Deadzone ----- //
	// hysteresis currently disabled because josh likes max speed better
	// to enable, multiply motor output by hysteresis_mult
	//private double deadzone = 0.2;

	// ---- Turn to Degree ---- //
	private boolean is_auto_turning = false;

	// ---- deltaTime ----//
	private double auto_start_time = 0;

	// ------ Gyro ------ //
	private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();

	// ------ Sonar ------ //
	public AnalogInput ultrasonicSensor = new AnalogInput(0);
	double voltageScaleFactor = 0;

	// ------ Camera ------ //
	private final UsbCamera camera;

	// ------ Shooter ----- //
	double time_at_press = 0.0;

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
			shooter.set( ShootingState.NOTHING, ShootingState.LAUNCH );
		} else {
			shooter.set( ShootingState.LAUNCH, ShootingState.LAUNCH );
		}

		servoPeriodic();

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

		// TODO: See if squaring the input makes controlling feel better.
		// xSpeed = Math.pow(xSpeed, 2) * Math.signum(xSpeed);
		// ySpeed = Math.pow(ySpeed, 2) * Math.signum(ySpeed);
		// rotSpeed = Math.pow(rotSpeed, 2) * Math.signum(rotSpeed);

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
			intake.manualSetIntakePosition(IntakePosition.IN);
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
		ShootingState shooterInState = ShootingState.NOTHING;
		ShootingState shooterOutState = ShootingState.NOTHING;
		
		/*
		if (controllerYellow.getAButton()) {
			shooterInState = ShootingState.LAUNCH;
		} else if (controllerYellow.getXButton()) {
			shooterInState = ShootingState.UNLAUNCH;
		}

		ShootingState shooterOutState = ShootingState.NOTHING;
		if (controllerYellow.getLeftBumperButton()) {
			shooterOutState = ShootingState.LAUNCH;
		} else if (controllerYellow.getXButton()) {
			shooterOutState = ShootingState.UNLAUNCH;
		}
		*/
		
		// this section should start and stop the motor with a single button press (not holding) 
		// and start the motor with a delay on the inside one

		double currentTime = Timer.getFPGATimestamp();

		if(controllerYellow.getXButtonPressed()) {
			if(shooterInState == ShootingState.LAUNCH) {
				shooterInState = ShootingState.UNLAUNCH;
				shooterOutState = ShootingState.UNLAUNCH;
			}
			else{
				shooterOutState = ShootingState.LAUNCH;
				time_at_press = currentTime;
			}
		}

		double elapsedTime = currentTime - time_at_press;

		// the 0.25 is the delay before the inside motor starts
		if(elapsedTime > 0.25 && shooterOutState == ShootingState.LAUNCH) {
			shooterInState = ShootingState.LAUNCH;
		}

		shooter.set(shooterInState, shooterOutState);
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
		// cleaning up to make it easieir to move to autonomousPeriodic
		initCommands(new ClimbExtendCmd( climb ), new ClimbRetractCmd( climb ));

		/*
		autoTestCmds.add( new ClimbExtendCmd( climb ) );
		autoTestCmds.add( new ClimbRetractCmd( climb ) );

		CommandBase firstCmd = autoTestCmds.get( 0 );
		firstCmd.commandInit();
		*/
	}

	public void testPeriodic2() {
		// cleaning up to make it easier to move this to autonomousPeriodic
		runCommands();

		/*
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
		*/
		
	}

	/**
	 * resets command array to the ones listed in the function parameter. Also use this for initialization.
	 * (this is just to clean up the code so theres not just a bunch of array.add everywhere)
	 */
	public void initCommands(CommandBase... commands) {
		// clears array and adds the function parameters to the array
		autoTestCmds.clear();
		for(int i = 0; i < commands.length; i++) {
			autoTestCmds.add(commands[i]);
		}

		// Sahil's code unchanged
		CommandBase firstCmd = autoTestCmds.get( 0 );
		firstCmd.commandInit();
	}

	/**use this in the autoPeriodic function to run the commands*/
	public void runCommands() {
		//Sahil's code unchanged

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

		/*
		// commented out because this deadzone will only work for swerve drive and cause problems for tank
		double joystickMagnitude = Math.sqrt(Math.pow(controllerRed.getLeftX(), 2) + Math.pow(controllerRed.getLeftY(), 2));
		if(deadzone > joystickMagnitude){
			throttle = 0.0;
		}
		*/
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
