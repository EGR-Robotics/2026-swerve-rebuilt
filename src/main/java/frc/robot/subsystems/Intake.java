package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller1 = new TalonFX(27, "5980");//changed ids, check in tuner
    private final TalonFX intakeRoller2 = new TalonFX(30, "5980");//changed ids, check in tuner
    private final TalonFX intakePivot = new TalonFX(26, "5980");//changed ids, check in tuner

    private static final double intakeSpeed1 = 3;
    private static final double intakeSpeed2 = 3;

    private static final double voltageNumber = 12;

    public Intake() {

        // Current Limits
        TalonFXConfiguration cfg1 = new TalonFXConfiguration();
        cfg1.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg1.CurrentLimits.SupplyCurrentLimit = 25;
        intakeRoller1.getConfigurator().apply(cfg1);

        TalonFXConfiguration cfg2 = new TalonFXConfiguration();
        cfg2.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg2.CurrentLimits.SupplyCurrentLimit = 25;
        intakeRoller2.getConfigurator().apply(cfg2);

        TalonFXConfiguration pivotCfg = new TalonFXConfiguration();
        pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotCfg.CurrentLimits.SupplyCurrentLimit = 20;
        intakePivot.getConfigurator().apply(pivotCfg);

        intakePivot.setControl(new VoltageOut(0));
    }

    public void intake() {
        intakeRoller1.setControl(new VoltageOut(-1 * intakeSpeed1 * voltageNumber));
        intakeRoller2.setControl(new VoltageOut(1 * intakeSpeed2 * voltageNumber));
    }

    public void reverseIntake() {
        intakeRoller1.setControl(new VoltageOut(intakeSpeed1 * voltageNumber));
        intakeRoller2.setControl(new VoltageOut(-intakeSpeed2 * voltageNumber));
    }

    public void stopRoller() {
        intakeRoller1.setControl(new VoltageOut(0));
        intakeRoller2.setControl(new VoltageOut(0));
    }

    public void lowerIntake() {
        intakePivot.setControl(new VoltageOut(-4));  // DOWN
        setBrakeMode(false);
    }

    public void raiseIntake() {
        intakePivot.setControl(new VoltageOut(4));   // UP
        setBrakeMode(false);
    }

    public void stopPivot() {
        intakePivot.setControl(new VoltageOut(0));
        setBrakeMode(false);
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
