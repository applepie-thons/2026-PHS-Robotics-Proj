// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.lang.FdLibm.Pow;
import java.util.Vector;

import javax.imageio.spi.IIOServiceProvider;
import javax.sound.sampled.Port;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Power;

import com.ctre.phoenix6.hardware.TalonFX;

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
    private final XboxController controllerGreen = new XboxController(1);

    // ------ Drivetrain Control ------ //
    private final WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);  // this one was initially 5 //
    private final WPI_TalonSRX rightDrive = new WPI_TalonSRX(3); // this one was 3, works //
    private final TalonFX kraken = new TalonFX(7);

    // encoder thingy
    private final Encoder testEncoder = new Encoder(
        1,
        0,
        false,
        Encoder.EncodingType.k2X
    );

    // ------ Gyro ------ //

    private DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);
    private double encoderRotations = 0.0;

    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private boolean autoTurn = false;
    /*
    private boolean is_auto_turning = false;
    private boolean first_auto_turn_call = true;
    private double auto_turn_direct = 1.0;
    */

    // private AHRS big_gyro = new AHRS();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {}

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */

    @Override
    public void robotInit() {
    rightDrive.setInverted(true);
    CameraServer.startAutomaticCapture(0);

    }


    @Override
    public void autonomousInit() {
        testEncoder.reset();
        // CameraServer.startAutomaticCapture();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        encoderRotations = testEncoder.getDistance();
        SmartDashboard.putNumber("test Encoder Value", encoderRotations);
        
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        m_gyro.reset();
    }
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double converted_gyro = Math.abs(m_gyro.getRotation2d().getDegrees() % 360.0);

        SmartDashboard.putNumber("gyroRotation", converted_gyro);

        double throttle = 0.5;
        double leftJoystick = controllerRed.getLeftY();
        double rightJoystick = controllerRed.getRightX();

        robotDrive.tankDrive(leftJoystick * throttle, rightJoystick * throttle);
        turnToOrigin(0.0, converted_gyro, 10);

        if (controllerRed.getAButtonPressed() && autoTurn == false) {
            robotDrive.tankDrive(0.5, -0.5);
        }

        /*
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
        */
        
        //this.leftDrive.set(-controllerRed.getLeftY()*speed);
    }

    private void turnToOrigin(double targetDegree, double currentDegree, double accuracy)
    {
        double degreeDiff = (targetDegree - currentDegree) % 360;
        if (-accuracy < degreeDiff && degreeDiff < accuracy) {
            autoTurn = false;
            return;
        }
    }

    /* 
    private void turn_to_degree(double current_degree, double target_degree, double speed, double accuracy) {
            double diff = (target_degree - current_degree) % 360;
            if(-accuracy < diff && diff < accuracy) {
                is_auto_turning = false;
                return;
            }
            SmartDashboard.putBoolean("first_auto_turn_call", first_auto_turn_call);
            if (first_auto_turn_call == true){
                auto_turn_direct = -(diff/Math.abs(diff));
                if(diff < -180 || diff > 180){
                    auto_turn_direct *= -1.0;
                }
                first_auto_turn_call = false;
            }
            
            
            SmartDashboard.putNumber("diff", diff);
            SmartDashboard.putNumber("direction", auto_turn_direct);
            robotDrive.arcadeDrive(0.0, auto_turn_direct * speed);
    }
    */

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
    public void testPeriodic() {
	
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
