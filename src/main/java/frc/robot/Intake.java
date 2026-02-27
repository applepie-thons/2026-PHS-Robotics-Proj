package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake
{
    private TalonFX intakeDrive;
    private TalonFX intakePivot1;
    private TalonFX intakePivot2;
    private boolean intakingState = false;

    private TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();


    public Intake(TalonFX intakeDrive, TalonFX intakePivot1, TalonFX intakePivot2) {
        this.intakeDrive = intakeDrive;
        this.intakePivot1 = intakePivot1;
        this.intakePivot2 = intakePivot2;

        //pid cofiguration for intakePivot1    
        Slot0Configs intakePivot1Controller = intakeConfigs.Slot0;
        intakePivot1Controller.kP = 0;
        intakePivot1Controller.kI = 0;
        intakePivot1Controller.kD = 0;
        intakePivot1Controller.kG = 0;
        this.intakePivot1.getConfigurator().apply(intakePivot1Controller);


        //pid configuration for intakePivot2
        Slot1Configs intakePivot2Controller = intakeConfigs.Slot1;
        intakePivot2Controller.kP = 0;
        intakePivot2Controller.kI = 0;
        intakePivot2Controller.kD = 0;
        intakePivot2Controller.kG = 0;
        this.intakePivot2.getConfigurator().apply(intakePivot1Controller);
    }

     
    public boolean getIntakingState() {
        return intakingState;
    }

    public void setIntakingState(boolean newState) {
        intakingState = newState;
    }

    private void setIntakePosition() {

    }

    public void runIntake() {
        if (intakingState == true) {
            intakeDrive.set(-0.65 );
        }
        else {
            intakeDrive.set(0);
        }
    }
}
