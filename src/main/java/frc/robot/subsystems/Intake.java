package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller = new TalonFX(27, "5980");//changed ids, check in tuner
    private final TalonFX intakePivot = new TalonFX(26, "5980");//changed ids, check in tuner

    private static final double intakeSpeed = 5;
    private static final double voltageNumber = 12;

    public Intake() {
        intakePivot.setControl(new VoltageOut(0));
    }

    public void intake(double triggerVal) {
        intakeRoller.setControl(new VoltageOut(-triggerVal * intakeSpeed * voltageNumber));
    }

    public void reverseIntake() {
        intakeRoller.setControl(new VoltageOut(intakeSpeed * voltageNumber));
    }

    public void stopRoller() {
        intakeRoller.setControl(new VoltageOut(0));
    }

    public void lowerIntake() {
        intakePivot.setControl(new VoltageOut(4));  // DOWN
        setBrakeMode(false);
    }

    public void raiseIntake() {
        intakePivot.setControl(new VoltageOut(-4));   // UP
        setBrakeMode(false);
    }

    public void stopPivot() {
        intakePivot.setControl(new VoltageOut(0));
        setBrakeMode(true);
    }

    public void setPivotVoltage(double volts) {
        intakePivot.setControl(new VoltageOut(volts));
        setBrakeMode(false);
    }


    public void stopAll() {
        stopRoller();
        stopPivot();
    }

    private void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        intakePivot.setNeutralMode(mode);
    }
}
