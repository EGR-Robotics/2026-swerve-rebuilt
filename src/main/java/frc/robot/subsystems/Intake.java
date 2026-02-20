package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller = new TalonFX(99);//change IDS
    private final TalonFX intakePivot = new TalonFX(99);

    private static final double intakeSpeed = 0.9;
    private static final double raiseIntakeSpeed = 0.9;

    public Intake() {
        // intakeRoller.setNeutralMode(NeutralModeValue.Brake);
        // intakePivot.setNeutralMode(NeutralModeValue.Brake);
    }

    /** Run intake forward */
    public void intake(double triggerVal) {
        intakeRoller.setControl(new VoltageOut(triggerVal * intakeSpeed * 12));
    }

    /** Reverse intake */
    public void reverseIntake() {
        intakeRoller.setControl(new VoltageOut(-intakeSpeed * 12));
    }

    /** Stop intake roller */
    public void stopRoller() {
        intakeRoller.setControl(new VoltageOut(0));
    }

    /** Lower intake (pivot down) */
    public void lowerIntake() {
        intakePivot.setControl(new VoltageOut(raiseIntakeSpeed * 12));
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
