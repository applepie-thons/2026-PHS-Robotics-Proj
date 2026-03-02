package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake
{
    private TalonFX intakeDrive;
    private TalonFX intakePivot1;
    private TalonFX intakePivot2;
    

    private boolean intakingState = false;
    private boolean intakeDown = false;

    enum intakePosition {
        REST(0),
        ACTIVE(0);

        private double position;
        intakePosition(double rotations) {
            position = rotations;
        }

        public double getPosition() {
            return position;
        }
    }

    private TalonFXConfiguration intakePivotConfig1 = new TalonFXConfiguration();
    private TalonFXConfiguration intakePivotConfig2 = new TalonFXConfiguration();

    public Intake(TalonFX intakeDrive, TalonFX intakePivot1, TalonFX intakePivot2) {
        this.intakeDrive = intakeDrive;
        this.intakePivot1 = intakePivot1;
        this.intakePivot2 = intakePivot2;

        //pid cofiguration for intakePivot1    
        Slot0Configs intakePivot1Controller = intakePivotConfig1.Slot0;
        intakePivot1Controller.kP = 0;
        intakePivot1Controller.kI = 0;
        intakePivot1Controller.kD = 0;
        intakePivot1Controller.kG = 1;
        this.intakePivot1.getConfigurator().apply(intakePivot1Controller);

        //pid configuration for intakePivot2
        Slot0Configs intakePivot2Controller = intakePivotConfig2.Slot0;
        intakePivot2Controller.kP = 0;
        intakePivot2Controller.kI = 0;
        intakePivot2Controller.kD = 0;
        intakePivot2Controller.kG = 1;
        this.intakePivot2.getConfigurator().apply(intakePivot2Controller);
    }

     
    public boolean getIntakingState() {
        return intakingState;
    }

    public TalonFX getPivot1() {
        return intakePivot1;
    }

    public void setIntakingState(boolean newState) {
        intakingState = newState;
    }

    private void setIntakePosition() {
        intakePivotConfig1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        final PositionVoltage voltageRequest1 = new PositionVoltage(0).withSlot(0);
        final PositionVoltage voltageRequest2 = new PositionVoltage(0).withSlot(0);

        if (intakeDown == true) {
            intakePivot1.setControl(voltageRequest1.withPosition(intakePosition.ACTIVE.getPosition()));
            intakePivot2.setControl(voltageRequest2.withPosition(intakePosition.ACTIVE.getPosition()));            
        }
        else {
            intakePivot1.setControl(voltageRequest1.withPosition(intakePosition.REST.getPosition()));
            intakePivot2.setControl(voltageRequest2.withPosition(intakePosition.REST.getPosition()));

        }
    }

    public void runIntake() {
        if (intakingState == true) {
            intakeDrive.set(-0.02 );
        }
        else {
            intakeDrive.set(0);
        }

        //setIntakePosition();
    }
}
