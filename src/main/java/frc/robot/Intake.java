package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake
{
    enum IntakePosition {
        IN(0.9),
        OUT(0);

        private double position;
        IntakePosition(double rotations) {
            position = rotations;
        }

        public double getPosition() {
            return position;
        }
        public void setPosition(IntakePosition newState) {
            position = newState.getPosition();
        }
    }

    private TalonFX intakeDrive;
    private TalonFX intakePivot1;
    private TalonFX intakePivot2;
    private IntakePosition intakeLocation = IntakePosition.IN;
    private boolean intakingState = false;

    private TalonFXConfiguration intakePivotConfig1 = new TalonFXConfiguration();
    final PositionVoltage voltageRequest1 = new PositionVoltage(0).withSlot(0);
    


    public Intake(int intakeDrive, int intakePivot1, int intakePivot2) {
        this.intakeDrive = new TalonFX(intakeDrive);
        this.intakePivot1 = new TalonFX(intakePivot1);
        this.intakePivot2 = new TalonFX(intakePivot2);

        

        //pid cofiguration for intakePivot1    
        Slot0Configs intakePivot1Controller = intakePivotConfig1.Slot0;
        intakePivot1Controller.kP = 0;
        intakePivot1Controller.kI = 0;
        intakePivot1Controller.kD = 0;
        intakePivot1Controller.kG = 0;
        intakePivot1Controller.GravityType = GravityTypeValue.Arm_Cosine;
        intakePivot1Controller.GravityArmPositionOffset = 0;

        intakePivotConfig1.Feedback.SensorToMechanismRatio = 4;
        this.intakePivot1.getConfigurator().apply(intakePivotConfig1);

        this.intakePivot2.setControl(new Follower(intakePivot1, MotorAlignmentValue.Opposed));
    }

     
    public boolean getIntakingState() {
        return intakingState;
    }

    public TalonFX getPivot1() {
        return intakePivot1;
    }

    public TalonFX getPivot2() {
        return intakePivot2;
    }

    public TalonFXConfiguration geTalonFXConfiguration() {
        return intakePivotConfig1;
    }

    public void getIntakePositions() {
        SmartDashboard.putNumber("intake Piv 1 rotations", intakePivot2.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("intake Piv 2 rotations", intakePivot1.getPosition().getValueAsDouble());
    }

    public void setIntakingState(boolean newState) {
        intakingState = newState;
    }

    private void setIntakePosition() {
        // intakePivotConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        if (intakeLocation.equals(IntakePosition.OUT)) {
            intakePivot1.setControl(voltageRequest1.withPosition(intakeLocation.getPosition()));
        }
        else if (intakeLocation.equals(IntakePosition.IN)) {
            intakePivot1.setControl(voltageRequest1.withPosition(intakeLocation.getPosition()));

        }
    }
  

    public void runIntake() {
         
        if (intakingState == true) {
            intakeDrive.set(-0.55 );
        }
        else {
            intakeDrive.set(0);
        }
    
        setIntakePosition();
        getIntakePositions();
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
