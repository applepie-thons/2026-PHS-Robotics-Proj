package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake
{
    enum IntakePosition {
        IN(0.25),
        OUT(0);

        private double position;
        IntakePosition(double rotations) {
            position = rotations;
        }
        public double getPosition() {
            return position;
        }
    }

    private TalonFX wheelDrive;
    private TalonFX pivot1;
    private TalonFX pivot2;
    private IntakePosition intakeLocation = IntakePosition.IN;
    private boolean intakingState = false;
    private boolean autoMode = true;

    private TalonFXConfiguration pivotConfig1 = new TalonFXConfiguration();
    final PositionVoltage voltageRequest1 = new PositionVoltage(0).withSlot(0);
    


    public Intake(int wheelDrive, int pivot1, int pivot2) {
        this.wheelDrive = new TalonFX(wheelDrive);
        this.pivot1 = new TalonFX(pivot1);
        this.pivot2 = new TalonFX(pivot2);
        this.pivot1.setPosition(0);
        this.pivot2.setPosition(0);


        //pid cofiguration for pivot1    
        Slot0Configs pivotPIDController = pivotConfig1.Slot0;
        pivotPIDController.kP = 0.7;
        pivotPIDController.kI = 0;
        pivotPIDController.kD = 0;
        pivotPIDController.kG = 0.8;
        pivotPIDController.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig1.Feedback.SensorToMechanismRatio = 4;
        pivotConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //setting positing limits
        pivotConfig1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakePosition.IN.getPosition();
        pivotConfig1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakePosition.OUT.getPosition();
        pivotConfig1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        for (int i = 0; i < 4; i++) {
            this.pivot1.getConfigurator().apply(pivotConfig1);
        }
        this.pivot2.setControl(new Follower(pivot1, MotorAlignmentValue.Opposed));
    }

     
    public boolean getIntakingState() {
        return intakingState;
    }

    public boolean getAutoMode() {
        return autoMode;
    }

    public TalonFX getPivot1() {
        return pivot1;
    }

    public void logIntakePositions() {
        SmartDashboard.putNumber("intake Piv 2 rotations", pivot2.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("intake Piv 1 rotations", pivot1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intake piv 2 power", pivot2.get());
        SmartDashboard.putNumber("intake piv 1 power", pivot1.get());
    }

    public void swapIntakingState() {
        intakingState = !intakingState;
    }

    public void swapPivotMode() {
        autoMode = !autoMode;
    }


    private void setWheelState() {
        if (intakingState == true) {
            wheelDrive.set(-0.55 );
        }
        else {
            wheelDrive.set(0);
        }
    }

    private void setIntakePosition() {
        pivot1.setControl(voltageRequest1.withPosition(intakeLocation.getPosition()));     
    }

    private void setIntakePosition(double speed) {
        pivot1.set(speed);
    }

    public void intakePeriodic() {
        setWheelState();
        setIntakePosition();
        logIntakePositions();
    }

    public void intakePeriodic(double pivotSpeed) {
        setWheelState();
        setIntakePosition(pivotSpeed);
        logIntakePositions();
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
