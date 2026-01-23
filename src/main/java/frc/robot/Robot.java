// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    // ------ Game Controller ------ //
    // IMPORTANT: Plug the red labelled controller into the *RED* port *FIRST*. And
    // plug the green labelled controller into the *GREEN* port *SECOND*.
    private final XboxController controllerRed = new XboxController(0);
    //private final XboxController controllerGreen = new XboxController(1);

    // ------ Drivetrain Control ------ //
    private final WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);  // this one was initially 5 //
    private final WPI_TalonSRX rightDrive = new WPI_TalonSRX(3); // this one was 3, works //

    private DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);


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

    // encoder thingy
    /* 
    private final Encoder testEncoder = new Encoder(
        1,
        0,
        false,
        Encoder.EncodingType.k2X
    );
    */

    // ------ Gyro ------ //
    private ADXRS450_Gyro adxrGyro = new ADXRS450_Gyro();
    private boolean is_auto_turning = false;
    private boolean first_auto_turn_call = true;
    private double auto_turn_direct = 1.0;

    private AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

    // uhhhhhhh utrasonic stuff by michael i just copied from the documentation cause its basically the same implementation
    private final MedianFilter m_filter = new MedianFilter(5);
    private final Ultrasonic m_ultrasonic = new Ultrasonic(0, 1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftDrive::set, rightDrive::set);
    private final PIDController m_pidController = new PIDController(0.001, 0.0, 0.0);
    

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot()
    {
        SendableRegistry.addChild(m_robotDrive, leftDrive);
        SendableRegistry.addChild(m_robotDrive, rightDrive);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

    }

    @Override
    public void robotInit() {
        // Setup for ADXR Gyro
        adxrGyro.reset();
        adxrGyro.calibrate();

        // Setup for NavX Gyro
        navxMxp.reset();

        // testEncoder.reset();

	    rightDrive.setInverted(true);
	    CameraServer.startAutomaticCapture(0);
    }

    @Override
    public void autonomousInit() {
        CameraServer.startAutomaticCapture();

        m_pidController.setSetpoint(3.0);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        //this was for the test encoder
        //encoderRotations = testEncoder.getDistance();
        //SmartDashboard.putNumber("test Encoder Value", encoderRotations);

        double measurement = m_ultrasonic.getRangeMM();
        double filteredMeasurement = m_filter.calculate(measurement);
        double pidOutput = m_pidController.calculate(filteredMeasurement);

        m_robotDrive.arcadeDrive(pidOutput, 0, false);
    }

    /** This function is called once when teleop is enabled. */
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

    /** This function is called periodically during operator control. */

    // *****************************************************************
    // Temporarily named to `teleopPeriodicTank` to test our new sensors
    // in the current `teleopPeriodic`.
    // *****************************************************************
    public void teleopPeriodic() {
        double adxrGyro_degrees = adxrGyro.getRotation2d().getDegrees();
        if(adxrGyro_degrees < 360.0 && adxrGyro_degrees > -360.0){
            adxrGyro_degrees += 360.0;
        }
        else if(adxrGyro_degrees < -360.0){
            adxrGyro_degrees += 360.0 * Math.abs(adxrGyro_degrees) / 360;
        }
        double converted_gyro = Math.abs(adxrGyro_degrees) % 360.0;

        SmartDashboard.putNumber("gyroRotation", converted_gyro);

        double throttle = 0.25;
        double leftJoystick = controllerRed.getLeftY();
        double rightJoystick = controllerRed.getRightX();

        frontLeftSpeedFX.set(throttle * leftJoystick);
        frontLeftDirectionFX.set(throttle * rightJoystick);

        StatusSignal<Angle> flSignal = frontLeftEncoder.getAbsolutePosition();
        Angle flAngle = flSignal.getValue();
        SmartDashboard.putString("FL Angle", flAngle.toString());


        /*
        turnToOrigin(0.0, converted_gyro, 10);

        if (controllerRed.getAButtonPressed() && autoTurn == false) {
            robotDrive.tankDrive(0.5, -0.5);
        }
        */

        if (controllerRed.getAButtonPressed() && is_auto_turning == false){
            is_auto_turning = true;
            first_auto_turn_call = true;
        }

        if (is_auto_turning == false) {
            robotDrive.arcadeDrive(leftJoystick * throttle, -rightJoystick * throttle);
        }
        else{
            turn_to_degree(converted_gyro, 0.0, 0.5, 10.0);
        }


        //this.leftDrive.set(-controllerRed.getLeftY()*speed);
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
            robotDrive.arcadeDrive(0.0, auto_turn_direct * speed);
    }


    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
