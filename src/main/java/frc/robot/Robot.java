// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

//import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match; (might need later)
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConsts;
import frc.robot.Intake.IntakePosition;
import frc.robot.Shooter.ShootingState;

public class Robot extends TimedRobot {
	private boolean commandMode = false;
	private int reversePIDModifier = 1;

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

	// ------ Sonar ------ //
	public AnalogInput ultrasonicSensor = new AnalogInput(0);
	double voltageScaleFactor = 0;

	// ------ Camera ------ //
	// private final UsbCamera camera;


	// Autonomous command sequence.
	ArrayList<CommandBase> autoTestCmds;
	private int currCmdIndex = 0;

	public Robot() {
		// TODO (Sahil): Check that is actually drops the camera's resolution.
		//this.camera = CameraServer.startAutomaticCapture();
		//this.camera.setPixelFormat(PixelFormat.kMJPEG);
		//this.camera.setResolution(320, 240);
		//this.camera.setFPS(30);

		autoTestCmds = new ArrayList<CommandBase>();
	}

	@Override
	public void robotPeriodic() {
		voltageScaleFactor = 5/RobotController.getVoltage5V();
	}

	@Override
	public void robotInit() {}

	@Override
	public void autonomousInit() {
		shooter.setShootingState(ShootingState.LAUNCH);
	}

	@Override
	public void autonomousPeriodic() {
		shooter.periodic();
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
		if(controllerYellow.getAButton()) {
			shooter.setShootingState(ShootingState.LAUNCH);
		} else if(controllerYellow.getXButton()) {
			shooter.setShootingState(ShootingState.UNLAUNCH);
		} else {
			shooter.setShootingState(ShootingState.NOTHING);
		}
		shooter.periodic();
	}

	@Override
	public void teleopInit() {
		shooter.setShootingState(ShootingState.NOTHING);
	}

	@Override
	public void teleopPeriodic() {
		redControllerPeriodic();
		yellowControllerPeriodic();
	}

	@Override
	public void testInit() {
		// Autonomous test code.
		// initCommands(new ClimbExtendCmd( climb ), new ClimbRetractCmd( climb ));
	}

	public void testPeriodic2() {
		// cleaning up to make it easier to move this to autonomousPeriodic

		SmartDashboard.putNumber("gyro degrees", swerve_drive.navxMxp.getRotation2d().getDegrees());
		SmartDashboard.putNumber("gyro radians", swerve_drive.navxMxp.getRotation2d().getRadians());

		if (controllerRed.getLeftBumperButtonPressed() ) {
			swerve_drive.resetGyro();
		}

		if (commandMode == false) {
			double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 3);
			double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 3);
			double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 3);

			if(!is_auto_turning){
				swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
			}
		}
		else if (commandMode == true) {
			runCommands();
		}

		if (controllerRed.getStartButtonPressed()) {
			initCommands(new TurnToCmd(swerve_drive, 90, 0.035));//new MoveToCmd(swerve_drive, inches_to_meters(114.26 + 1.25), -1.0, 0.0), new MoveToCmd(swerve_drive, inches_to_meters(30.915), 0.0, 1.0));
			commandMode = !commandMode;
		}


		if (controllerRed.getBackButtonPressed()) {
			reversePIDModifier *= -1;
		}

		if (controllerRed.getXButtonPressed()) {
			swerve_drive.setKP(swerve_drive.kP += 0.05 * reversePIDModifier);
		}
		if (controllerRed.getAButtonPressed()) {
			swerve_drive.setKP(swerve_drive.kP += 0.1 * reversePIDModifier);
		}
		if (controllerRed.getBButtonPressed()) {
			swerve_drive.setKP(swerve_drive.kP += 1 * reversePIDModifier);
		}

		if (controllerRed.getYButtonPressed()) {
			swerve_drive.setKP(swerve_drive.kP += 2 * reversePIDModifier);
		}

		SmartDashboard.putNumber("kP", swerve_drive.kP);
		SmartDashboard.putNumber("reversePID", reversePIDModifier);

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
		currCmdIndex = 0;
		autoTestCmds.clear();
		for(int i = 0; i < commands.length; i++) {
			autoTestCmds.add(commands[i]);
		}

		CommandBase firstCmd = autoTestCmds.get( 0 );
		firstCmd.commandInit();
	}

	/**use this in the autoPeriodic function to run the commands*/
	public void runCommands() {
		if(currCmdIndex == autoTestCmds.size()) {
			// We have iterated through all of the commands. There's nothing left to do,
			// so return early.
			return;
		}
		CommandBase currCmd = autoTestCmds.get( currCmdIndex );
		boolean cmdFinished = currCmd.commandPeriodic();

		if (cmdFinished) {
			currCmdIndex += 1;
			if (currCmdIndex == autoTestCmds.size()) {
				return;
			}
			CommandBase nextCmd = autoTestCmds.get( currCmdIndex );
			nextCmd.commandInit();
		}
	}


	public void testPeriodic() {
		if (controllerRed.getStartButtonPressed()) {
			intake.swapPivotMode();
		}

		if (intake.getAutoMode() == true) {
			intake.intakePeriodic();
		}
		else {
			intake.intakePeriodic(0);
		}

		if (controllerRed.getLeftBumperButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.IN);
		}
		else if (controllerRed.getRightBumperButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.OUT);
		}

		if (controllerRed.getBackButtonPressed()) {
			reversePIDModifier *= -1;
		}
		if (controllerRed.getXButtonPressed()) {
			intake.addTok(0.05 * reversePIDModifier);
		}
		if (controllerRed.getAButtonPressed()) {
			intake.addTok(0.1 * reversePIDModifier);
		}
		if (controllerRed.getBButtonPressed()) {
			intake.addTok(0.5 * reversePIDModifier);
		}
		if (controllerRed.getYButtonPressed()) {
			intake.addTok(1 * reversePIDModifier);
		}

		SmartDashboard.putNumber("reversePID", reversePIDModifier);
		intake.logIntake();

		/*
		// commented out because this deadzone will only work for swerve drive and cause problems for tank
		double joystickMagnitude = Math.sqrt(Math.pow(controllerRed.getLeftX(), 2) + Math.pow(controllerRed.getLeftY(), 2));
		if(deadzone > joystickMagnitude){
			throttle = 0.0;
		}
		*/
	}

	public double inches_to_meters(double inches){
		return(inches * 0.0254);
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
