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

	enum AutoStartPos {
		LEFT,
		CENTER,
		RIGHT;
	}

	private AutoStartPos start_position = AutoStartPos.CENTER;

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
	double voltage_scale_factor = 0;
	
	enum to_wall_sonar_state
	{
		MOVING,
		DISABLED;
	}

	to_wall_sonar_state current_sonar_state = to_wall_sonar_state.DISABLED;

	// ----- Distance Test ------ //
	double dist_modifier_mult = 1.0;
	double dist_modifier = 1.65;
	double modifier_change_speed = 0.1;

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
		voltage_scale_factor = 5/RobotController.getVoltage5V();
	}

	@Override
	public void robotInit() {}

	@Override
	public void autonomousInit() {
		//shooter.setShootingState(ShootingState.LAUNCH);
		initCommandsFromStartPos(start_position);
	}

	@Override
	public void autonomousPeriodic() {
		//shooter.periodic();
		runCommands();
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

		// Square the input for finer grain control when the stick is partially held.
		xSpeed = Math.pow(xSpeed, 2) * Math.signum(xSpeed);
		ySpeed = Math.pow(ySpeed, 2) * Math.signum(ySpeed);
		rotSpeed = Math.pow(rotSpeed, 2) * Math.signum(rotSpeed);

		if(!is_auto_turning){
			swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
		}

		// ------------------------------- Climb ------------------------------- //
		// Square the inputs for finer-grain control.
		double climbRetractSpeed = Math.pow(controllerRed.getLeftTriggerAxis(), 2);
		double climbExtendSpeed = Math.pow(controllerRed.getRightTriggerAxis(), 2);
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
			SmartDashboard.putNumber("D-pad value", controllerRed.getPOV());
			is_auto_turning = true;
			// I dont know why this is commented out, but i'm going to keep the new stuff here like this
			// just in case 

			/*
			swerve_drive.setModules(
				controllerRed.getLeftX(),
				controllerRed.getLeftY(),
				swerve_drive.turn_to_degree_return_mode(controllerRed.getPOV(), 2));
				*/
			swerve_drive.turn_to_degree(controllerRed.getPOV(), 0.5);
		}
		else{
			is_auto_turning = false;
		}
		SmartDashboard.putNumber("gyro degrees", -swerve_drive.navxMxp.getRotation2d().getDegrees());
	}

	public void yellowControllerPeriodic() {
		// ------------------------------- Intake ------------------------------- //
		// Set desired speed of the intaking wheels.
		if (controllerYellow.getBButton()) {
			intake.setIntakeSpeed(-1);
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

	// BY SAHIL: More late night testing of command sequencing. Feel free to transfer
	// the command list to another place and remove this function.
	public void testPeriodic() {
		if (controllerRed.getLeftBumperButtonPressed() ) {
			swerve_drive.resetGyro();
			swerve_drive.resetSpeedEncoders();
		}

		if (commandMode) {
			runCommands();
		} else {
			redControllerPeriodic();
			/*
			double xSpeed = controllerRed.getLeftX() * (DriveConsts.maxMetersPerSecToMotorSpeed / 6);
			double ySpeed = controllerRed.getLeftY() * (DriveConsts.maxMetersPerSecToMotorSpeed / 6);
			double rotSpeed = controllerRed.getRightX() * (DriveConsts.maxRadPerSecToMotorSpeed / 6);
			swerve_drive.setModules(ySpeed, xSpeed, rotSpeed);
			*/
		}

		if (controllerRed.getRightBumperButtonPressed()) {
			// change the start_position variable to whatever side we are on before the match
			initCommandsFromStartPos(start_position);
			commandMode = !commandMode;
		}

		// distance modifier change code ------------------------
		if(controllerYellow.getStartButtonPressed()) {
			dist_modifier_mult *= -1.0;
		}

		if(controllerYellow.getAButtonPressed()) {
			dist_modifier += modifier_change_speed * dist_modifier_mult;
		}

		if(controllerYellow.getBackButtonPressed()) {
			if(modifier_change_speed == 0.1) {
				modifier_change_speed = 0.25;
			}
			else if(modifier_change_speed == 0.25) {
				modifier_change_speed = 1.0;
			}
			else if(modifier_change_speed == 1.0) {
				modifier_change_speed = 2.0;
			} else modifier_change_speed = 0.1;
			

			
		}

		SmartDashboard.putNumber("dist modifier", dist_modifier);
		SmartDashboard.putNumber("modifier change speed", modifier_change_speed);
		SmartDashboard.putNumber("dist modifier mult", dist_modifier_mult);

		swerve_drive.log();
		SmartDashboard.putBoolean("commandMode", commandMode);
		SmartDashboard.putNumber( "currCmdIndex", currCmdIndex);
		SmartDashboard.putNumber("gyro degrees", swerve_drive.navxMxp.getRotation2d().getDegrees());
	}

	// BY DAVID: For tuning PID with TurnToCmd and MoveToCmd.
	public void testPeriodic2() {
		sonarPeriodic();

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

	public void initCommandsFromStartPos(AutoStartPos start_pos) {
		if(start_pos == AutoStartPos.LEFT) {
			/*
			//untested; from against the hub
			initCommands(new MoveToCmd(swerve_drive, inches_to_meters(112.26 - 10), -1.0, 0.0, false),
						 new TurnToCmd(swerve_drive, 360, 0.035),
						 // new ClimbExtendCmd(climb),
						 new MoveToCmd(swerve_drive, inches_to_meters(14.3325 + dist_modifier), 0.0, -1.0, false));
			*/
		}
		else if(start_pos == AutoStartPos.RIGHT) {
			//tested; from middle of hump
			/*
			initCommands(new MoveToCmd(swerve_drive, inches_to_meters(80.76), -1.0, 0.0));
						 // new MoveToCmd(swerve_drive, inches_to_meters(66.29), 0.0, 1.0),
						 // new ClimbExtendCmd(climb),
						 // new MoveToCmd(swerve_drive, inches_to_meters(14), -1.0, 0.0),
						 // new ClimbRetractCmd(climb));
						 */
			//from outside edge of hump
			initCommands(new MoveToCmd(swerve_drive, 2.687, -1.0, 0.0, false),
						 new MoveToCmd(swerve_drive, dist_modifier + inches_to_meters(20), 0.0, 1.0, true),
						 new ClimbExtendCmd(climb),
						 new MoveToCmd(swerve_drive, 0.196 + inches_to_meters(3), 1.0, 0.0, false),
						 new ClimbRetractCmd(climb));
		}
		else {
			//for the center, just shooting
			initCommands(new MoveToCmd(swerve_drive, inches_to_meters(48 + 10), -1.0, 0.0, false), 
						 new ShootCmd(shooter, 12));
		}
	}

	// BY SUNAY: For tuning the intake arm.
	public void testPeriodic1() {
		if (controllerYellow.getStartButtonPressed()) {
			intake.swapPivotMode();
		}

		if (intake.getAutoMode() == true) {
			intake.intakePeriodic();
		}
		else {
			intake.intakePeriodic(0);
		}

		if (controllerYellow.getLeftBumperButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.IN);
		}
		else if (controllerYellow.getRightBumperButtonPressed()) {
			intake.manualSetIntakePosition(IntakePosition.OUT);
		}

		if (controllerYellow.getBackButtonPressed()) {
			reversePIDModifier *= -1;
		}
		if (controllerYellow.getXButtonPressed()) {
			intake.addTok(0.05 * reversePIDModifier);
		}
		if (controllerYellow.getAButtonPressed()) {
			intake.addTok(0.1 * reversePIDModifier);
		}
		if (controllerYellow.getBButtonPressed()) {
			intake.addTok(0.5 * reversePIDModifier);
		}
		if (controllerYellow.getYButtonPressed()) {
			intake.addTok(1 * reversePIDModifier);
		}

		SmartDashboard.putNumber("reversePID", reversePIDModifier);
		intake.logIntake();

		/*
		// commented out because this deadzone will only work for swerve drive and cause problems for tank
		double joystickMagnitude = Math.sqrt(Math.pow(controllerYellow.getLeftX(), 2) + Math.pow(controllerYellow.getLeftY(), 2));
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

	// hey sonar isnt actually so bad - Michael 2026
	public void sonarPeriodic() {
		double sensorRange = ultrasonicSensor.getVoltage() * voltage_scale_factor;

		double sensorInches = sensorRange * 39.3442622951;
		double sensorCentimeters = sensorInches * 2.54;

		SmartDashboard.putNumber("Sensor Range", sensorRange);
		SmartDashboard.putNumber("Sensor Range (Inches)", sensorInches);
		SmartDashboard.putNumber("Sensor Range (Centimeters)", sensorCentimeters);

		if (controllerRed.getRightBumperButtonPressed())
		{
			if (current_sonar_state == to_wall_sonar_state.DISABLED)
			{
				current_sonar_state = to_wall_sonar_state.MOVING;
			}
			else
			{
				current_sonar_state = to_wall_sonar_state.DISABLED;
			}
		}

		
		if (current_sonar_state == to_wall_sonar_state.MOVING)
		{
			double current_sonar_dir = swerve_drive.navxMxp.getRotation2d().getRadians();
			double x_dir = -Math.cos(current_sonar_dir);
			double y_dir = -Math.sin(current_sonar_dir);

			if (sensorInches < 36)
			{
				//swerve_drive.setModules(x_dir, y_dir , );
			}
		}
	}
}

/*
 * @Override
 * public void simulationInit() {}
 *
 * @Override
 * public void simulationPeriodic() {}
 */
