package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller1 = new TalonFX(27, "5980");
    private final TalonFX intakeRoller2 = new TalonFX(30, "5980");

    private static final double intakeSpeed1 = 3;
    private static final double intakeSpeed2 = 3;
    private static final double voltageNumber = 12;

    public Intake() {
        // Current limits
        TalonFXConfiguration cfg1 = new TalonFXConfiguration();
        cfg1.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg1.CurrentLimits.SupplyCurrentLimit = 25;
        intakeRoller1.getConfigurator().apply(cfg1);

        TalonFXConfiguration cfg2 = new TalonFXConfiguration();
        cfg2.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg2.CurrentLimits.SupplyCurrentLimit = 25;
        intakeRoller2.getConfigurator().apply(cfg2);
    }

    public void intake() {
        intakeRoller1.setControl(new VoltageOut(-intakeSpeed1 * voltageNumber));
        intakeRoller2.setControl(new VoltageOut(intakeSpeed2 * voltageNumber));
    }

    public void reverse() {
        intakeRoller1.setControl(new VoltageOut(intakeSpeed1 * voltageNumber));
        intakeRoller2.setControl(new VoltageOut(-intakeSpeed2 * voltageNumber));
    }

    public void stop() {
        intakeRoller1.setControl(new VoltageOut(0));
        intakeRoller2.setControl(new VoltageOut(0));
    }
}
