package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake
{
    private TalonFX intakeDrive;
    private TalonFX intakePivot1;
    private TalonFX intakePivot2;
    private boolean intakingState = false;

    public Intake(TalonFX intakeDrive, TalonFX intakePivot1, TalonFX intakePivot2) {
        this.intakeDrive = intakeDrive;
        this.intakePivot1 = intakePivot1;
        this.intakePivot2 = intakePivot2;
    }

     
    public void setIntakingState(boolean newState) {
        intakingState = newState;
    }

    public void runIntake() {
        if (intakingState == true) {
            intakeDrive.set(-1);
        }
        else {
            intakeDrive.set(0);
        }
    }
}
