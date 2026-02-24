package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller = new TalonFX(27, "5980");
    private final TalonFX intakePivot = new TalonFX(26, "5980");

    private static final double intakeSpeed = 0.9;//TODO: Change the speeds.
    private static final double lowerIntakeSpeed = 0.9;
    private static final double raiseIntakeSpeed = -0.9;
    private static final double voltageNumber = 12;

    public Intake() {
        // intakeRoller.setNeutralMode(NeutralModeValue.Brake);
        // intakePivot.setNeutralMode(NeutralModeValue.Brake);
    }

    /** Run intake forward */
    public void intake(double triggerVal) {
        intakeRoller.setControl(new VoltageOut(-triggerVal * intakeSpeed * voltageNumber)); 
    }

    /** Reverse intake */
    public void reverseIntake() {
        intakeRoller.setControl(new VoltageOut(intakeSpeed * voltageNumber));
    }

    /** Stop intake roller */
    public void stopRoller() {
        intakeRoller.setControl(new VoltageOut(0)); 
    }

    /** Lower intake (pivot down) */
    public void lowerIntake() {
        intakePivot.setControl(new VoltageOut(lowerIntakeSpeed * voltageNumber));
    }

    public void raiseIntake() {
        intakePivot.setControl(new VoltageOut(raiseIntakeSpeed * voltageNumber));
    }

    /** Stop pivot motor */
    public void stopPivot() {
        intakePivot.setControl(new VoltageOut(0));
    }

    /** Stop everything */
    public void stopAll() {
        stopRoller();
        stopPivot();
    }
}
