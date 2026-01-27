// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    // ------ Game Controller ------ //
    // IMPORTANT: Plug the red labelled controller into the *RED* port *FIRST*. And
    // plug the green labelled controller into the *GREEN* port *SECOND*.
    private final XboxController controllerRed = new XboxController(0);

    // ------ Swerve Control ------ //
    private static TalonFX frontLeftSpeedFX = new TalonFX(1);
    private static TalonFX frontRightSpeedFX = new TalonFX(4);
    private static TalonFX backLeftSpeedFX = new TalonFX(7);
    private static TalonFX backRightSpeedFX = new TalonFX(10);

    private static TalonFX frontLeftDirectionFX = new TalonFX(2);
    private static TalonFX frontRightDirectionFX = new TalonFX(5);
    private static TalonFX backLeftDirectionFX = new TalonFX(8);
    private static TalonFX backRightDirectionFX = new TalonFX(11);

    private static CoreCANcoder frontLeftEncoder = new CoreCANcoder(0);
    private static CoreCANcoder frontRightEncoder = new CoreCANcoder(3);
    private static CoreCANcoder backLeftEncoder = new CoreCANcoder(6);
    private static CoreCANcoder backRightEncoder = new CoreCANcoder(9);

    int currMotor = 0;
    TalonFX allDirectionMotors[] = {
		frontLeftDirectionFX, frontRightDirectionFX,
		backLeftDirectionFX, backRightDirectionFX};
    TalonFX allSpeedMotors[] = {
		frontLeftSpeedFX, frontRightSpeedFX,
		backLeftSpeedFX, backRightSpeedFX};
	CoreCANcoder allWheelEncoders[] = {
		frontLeftEncoder, frontRightEncoder,
		backLeftEncoder, backRightEncoder};

    // encoder thingy
    /*
	  private final Encoder testEncoder = new Encoder(
	  1,
	  0,
	  false,
	  Encoder.EncodingType.k2X
	  );
    */
    // ----- Deadzone + Hysteresis ----- // 
    private double deadzone = 0.2;
    private double last_update_stickMagnitude = 0.0; // magnitude of the joystick on last update

    // ------ Gyro ------ //
    private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();
    private boolean is_auto_turning = false;
    private boolean first_auto_turn_call = true;
    private double auto_turn_direct = 1.0;
    

    private AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

    // uhhhhhhh ultrasonic stuff by michael i just copied from the documentation cause its basically the same implementation
    private final MedianFilter m_filter = new MedianFilter(5);
    private final Ultrasonic m_ultrasonic = new Ultrasonic(0, 1);
    private final PIDController m_pidController = new PIDController(0.001, 0.0, 0.0);

    public Robot() {}

    @Override
    public void robotPeriodic() {}

    @Override
    public void robotInit() {
        // Setup for ADXR Gyro
        adxrGyro.reset();
        adxrGyro.calibrate();

        // Setup for NavX Gyro
        navxMxp.reset();

        // testEncoder.reset();

	    CameraServer.startAutomaticCapture(0);
    }

    @Override
    public void autonomousInit() {
        CameraServer.startAutomaticCapture();

        m_pidController.setSetpoint(3.0);
    }

    @Override
    public void autonomousPeriodic() {
        //this was for the test encoder
        //encoderRotations = testEncoder.getDistance();
        //SmartDashboard.putNumber("test Encoder Value", encoderRotations);

        double measurement = m_ultrasonic.getRangeMM();
        double filteredMeasurement = m_filter.calculate(measurement);
        double pidOutput = m_pidController.calculate(filteredMeasurement);
    }

    @Override
    public void teleopInit() {}

    /*
	  @Override
	  public void teleopPeriodic() {
	  // Getting "Distance travelled" from kraken (needs to be converted to a useful unit).
	  StatusSignal<Angle> signal = kraken.getPosition();
	  Angle angle = signal.getValue();
	  Shuffleboard.getTab("Sensors").add(m_ultrasonic);
	  SmartDashboard.putNumber("baseUnit", angle.baseUnitMagnitude());
	  SmartDashboard.putString("toString", angle.toString());

	  // Getting data from original gyro.
	  double gyroDegrees = adxrGyro.getRotation2d().getDegrees();
	  SmartDashboard.putNumber("gryoDegrees", gyroDegrees);

	  // Getting angle from NavX
	  double navXAngle = navxMxp.getAngle();
	  SmartDashboard.putNumber("gryo2Degrees", navXAngle);
	  }
	*/

	public double getAdxrGyro() {
		double adxrGyro_degrees = adxrGyro.getRotation2d().getDegrees();
        if(adxrGyro_degrees < 360.0 && adxrGyro_degrees > -360.0){
            adxrGyro_degrees += 360.0;
        }
        else if(adxrGyro_degrees < -360.0){
            adxrGyro_degrees += 360.0 * Math.abs(adxrGyro_degrees) / 360;
        }
        return Math.abs(adxrGyro_degrees) % 360.0;
	}

	public double wheelEncoderToSwerveInput(CoreCANcoder encoder) {
		double angleRadians = encoder.getAbsolutePosition().getValue().baseUnitMagnitude();
		return Math.toDegrees(angleRadians * -1);
	}

    /** This function is called periodically during operator control. */
    // *****************************************************************
    // Temporarily named to `teleopPeriodicTank` to test our new sensors
    // in the current `teleopPeriodic`.
    // *****************************************************************
    public void teleopPeriodic() {
		double convertedGyro = getAdxrGyro();
        SmartDashboard.putNumber("gyroRotation", convertedGyro);

        double throttle = 0.25;
        double leftJoystickX = controllerRed.getLeftX();
        double leftJoystickY = controllerRed.getLeftY();
        double rightJoystick = controllerRed.getRightX();

        double joystickMagnitude = Math.sqrt(Math.pow(leftJoystickX, 2) + Math.pow(leftJoystickY, 2));
        
        /* commented out because this deadzone will only work for swerve drive and cause problems for tank
        if(deadzone > joystickMagnitude){
          throttle = 0.0;
        }
        */

	    if(controllerRed.getRightBumperButtonPressed()) {
	        currMotor = (currMotor + 1) % 4;
	    }
	    SmartDashboard.putNumber("currMotor", currMotor);

	    double reverseSpeedDirection = 1;
	    if(currMotor == 0 || currMotor == 2) {
			reverseSpeedDirection = -1;
	    }

        allSpeedMotors[currMotor].set(throttle * leftJoystickY * reverseSpeedDirection);
	    allDirectionMotors[currMotor].set(throttle * rightJoystick);

		for(int i = 0; i < allWheelEncoders.length; i++) {
			SmartDashboard.putNumber( "Motor " + i + " Angle",
									  wheelEncoderToSwerveInput(allWheelEncoders[i]));
		}
        /*
		  turnToOrigin(0.0, convertedGyro, 10);

		  if (controllerRed.getAButtonPressed() && autoTurn == false) {
		  robotDrive.tankDrive(0.5, -0.5);
		  }

        if (controllerRed.getAButtonPressed() && is_auto_turning == false){
            is_auto_turning = true;
            first_auto_turn_call = true;
        }

        if (is_auto_turning == false) {
            robotDrive.arcadeDrive(leftJoystick * throttle, -rightJoystick * throttle);
        }
        else{
            turn_to_degree(convertedGyro, 0.0, 0.5, 10.0);
        }
		*/


    // keep at bottom so it only updates at the end and is usable in entire function
    last_update_stickMagnitude = joystickMagnitude;
    }

    private void turn_to_degree(double current_degree, double target_degree, double speed, double accuracy) {
		double diff = (target_degree - current_degree) % 360;
		if(-accuracy < diff && diff < accuracy) {
			is_auto_turning = false;
			return;
		}
		if (first_auto_turn_call == true){
			auto_turn_direct = diff/Math.abs(diff);
			if(diff < -180 || diff > 180){
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
  // Unused function headers.
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
*/
