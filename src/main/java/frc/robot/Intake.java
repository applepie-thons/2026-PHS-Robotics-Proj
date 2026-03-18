package frc.robot;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ConfigConsts;

/**
 * Models an intake that uses three talon fx devices, one to suck in items
 * and two for pivoting the mechanism.
 *
 * @author Sunay Patel
 * @version March, 2026
 */
public class Intake
{
    //TODO get real values for in and out positions
    enum IntakePosition {
        START(0),
        IN(-0.11),
        PARTIAL(-0.25),
        OUT(-0.33);

        private double position;
        IntakePosition(double rotations) {
            position = rotations;
        }
    }

    private TalonFX wheelDrive;
    private TalonFX pivot1;
    private TalonFX pivot2;

    private IntakePosition intakeLocation = IntakePosition.START;
    //private boolean intakingState = false;
    private boolean autoMode = false;
    private double intakeSpeed = 0;

    private TalonFXConfiguration pivotConfig1 = new TalonFXConfiguration();
    private final PositionVoltage voltageRequest = new PositionVoltage(0).withSlot(0);


    public Intake() {
        this.wheelDrive = new TalonFX(ConfigConsts.intakeWheelDriveMotorId);
        this.pivot1 = new TalonFX(ConfigConsts.intakePivot1MotorId);
        this.pivot2 = new TalonFX(ConfigConsts.intakePivot2MotorId);
        this.pivot1.setPosition(0);
        this.pivot2.setPosition(0);

        //pid cofiguration for pivot1
        pivotConfig1.Slot0.kP = 8; //needs to be redone 3rd
        pivotConfig1.Slot0.kI = 0; //do this last
        pivotConfig1.Slot0.kD = 1.65; //needs to be set 4th
        pivotConfig1.Slot0.kG = 0.5; //needs to be redone 1st
        pivotConfig1.Slot0.kS = 0; //needs to be set 2nd
        pivotConfig1.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        //editing sensor data for calculations
        pivotConfig1.Feedback.SensorToMechanismRatio = 5;
        pivotConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //setup for current limits
        pivotConfig1.CurrentLimits.StatorCurrentLimit = 40;
        pivotConfig1.CurrentLimits.StatorCurrentLimitEnable = true;

        //setting positioning limits
        pivotConfig1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePosition.IN.position;
        pivotConfig1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePosition.OUT.position;
        pivotConfig1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        //applying configuration
        StatusCode configStatus = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            configStatus = this.pivot1.getConfigurator().apply(pivotConfig1);
            if (configStatus.isOK()) {
                System.out.println("Configuration Sucessfully Applied");
            }
        }
        if (!configStatus.isOK()) {
            System.out.println("Could not apply configuration, error code: " + configStatus.toString());
        }
        SmartDashboard.putBoolean("Configuration Applied", configStatus.isOK());

        this.pivot2.setControl(new Follower( ConfigConsts.intakePivot1MotorId,
											 MotorAlignmentValue.Opposed));
    }

    public boolean getAutoMode() {
        return autoMode;
    }

    public void swapPivotMode() {
        autoMode = !autoMode;
    }

    public void setIntakeSpeed(double newSpeed) {
        intakeSpeed = newSpeed;
    }

    //Only use for testing
    public void manualSetIntakePosition(IntakePosition newPosition) {
        intakeLocation = newPosition;
    }

    //TODO if statements here must be edited once proper positions have been implemented
    private void setIntakePosition() {
        //intakeLocation = (intakingState) ? IntakePosition.OUT : IntakePosition.IN;
        pivot1.setControl(voltageRequest.withPosition(intakeLocation.position));
    }

    private void setIntakePosition(double speed) {
        pivot1.set(speed);
    }

    public void intakePeriodic() {
        wheelDrive.set(intakeSpeed);
        setIntakePosition();
        logIntake();
    }

    public void intakePeriodic(double pivotSpeed) {
        wheelDrive.set(intakeSpeed);
        setIntakePosition(pivotSpeed);
        logIntake();
    }


    public void logIntake() {
        SmartDashboard.putBoolean("Automode", autoMode);
        SmartDashboard.putBoolean("Intaking State", intakeSpeed != 0);
        SmartDashboard.putNumber("intake Piv 2 rotations", pivot2.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("intake Piv 1 rotations", pivot1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intake piv 2 power", pivot2.get());
        SmartDashboard.putNumber("intake piv 1 power", pivot1.get());
        SmartDashboard.putNumber("Intake velocity", pivot1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Error", intakeLocation.position - pivot1.getPosition().getValueAsDouble());
    }
}


// Notes
/*
        Values that should be set to send out the intake

        if (controllerRed.getYButton()) {
			intake.getPivot1().set(0.1);
			intake.getPivot2().set(-0.1);
		}
		else {
			intake.getPivot1().set(0);
			intake.getPivot2().set(0);
		}
*/

/*
    This is a valid way to check what the current intakeLocation is.

public void enumTest() {
        if (intakeLocation.equals(IntakePosition.OUT)) {
            SmartDashboard.putNumber("intakeLocationOut", intakeLocation.getPosition());
        }
        else if (intakeLocation.equals(IntakePosition.IN)) {
            SmartDashboard.putNumber("intakeLocationIn", intakeLocation.getPosition());
        }
    }
 */

 /*
    This can allow for a smaller switch case, but is arguably less readable

    if (pivot1.getPosition().getValueAsDouble() >= intakeLocation.position - 0.05) {
        pivot1.stopMotor();
        break;
    }
    pivot1.setControl(voltageRequest.withPosition(intakeLocation.position));
*/


/*
    Springfield 12:30 a.m. I don't have time to document
    public void setWheelSpeed() {
        wheelDrive.set(intakeSpeed);
         double speed = (intakingState) ? -0.55 : 0;
        wheelDrive.set(speed);

 */
