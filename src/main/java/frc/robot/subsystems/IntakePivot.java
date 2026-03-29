package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

    private final TalonFX intakePivot = new TalonFX(26, "5980");

    public IntakePivot() {
        // Current limit
        TalonFXConfiguration pivotCfg = new TalonFXConfiguration();
        pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotCfg.CurrentLimits.SupplyCurrentLimit = 20;
        intakePivot.getConfigurator().apply(pivotCfg);

        // Prevent movement on enable
        intakePivot.setControl(new VoltageOut(0));
    }

    public void raiseIntake() {
        intakePivot.setControl(new VoltageOut(4));
        setBrakeMode(false);
    }

    public void lowerIntake() {
        intakePivot.setControl(new VoltageOut(-4));
        setBrakeMode(false);
    }

    public void setPivotVoltage(double volts) {
        intakePivot.setControl(new VoltageOut(volts));
        setBrakeMode(false);
    }

    public void stop() {
        intakePivot.setControl(new VoltageOut(0));
        setBrakeMode(false);
    }

    private void setBrakeMode(boolean brake) {
        intakePivot.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
