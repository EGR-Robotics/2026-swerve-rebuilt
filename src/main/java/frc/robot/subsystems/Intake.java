package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller = new TalonFX(27, "5980");
    private final TalonFX intakePivot = new TalonFX(26, "5980");
    private final CANcoder pivotEncoder = new CANcoder(9, "5980");

    private final PositionVoltage pivotRequest = new PositionVoltage(0);

    private static final double intakeSpeed = 1.5;
    private static final double voltageNumber = 12;
    private static final double hysteresis = 1.0; // degrees

    // TODO: Verify these with your actual CANcoder readings
    // Read pivotEncoder.getAbsolutePosition() in Tuner X with intake fully UP and fully DOWN
    private static final double pivotUpPosition   = 0.737549; // CANcoder reading when intake is UP
    private static final double pivotDownPosition = 0.767090; // CANcoder reading when intake is DOWN

    public Intake() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = 9;

        cfg.Slot0.kP = 6.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        intakePivot.getConfigurator().apply(cfg);
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
        if (isIntakeDown()) return;
        intakePivot.setControl(pivotRequest.withPosition(pivotDownPosition));
    }

    public void raiseIntake() {
        if (isIntakeUp()) return;
        intakePivot.setControl(pivotRequest.withPosition(pivotUpPosition));
    }

    public void stopPivot() {
        intakePivot.setControl(new VoltageOut(0));
    }

    public void stopAll() {
        stopRoller();
        stopPivot();
    }

    private double getPivotPosition() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    private boolean isIntakeUp() {
        return Math.abs(getPivotPosition() - pivotUpPosition) < (hysteresis / 360.0);
    }

    private boolean isIntakeDown() {
        return Math.abs(getPivotPosition() - pivotDownPosition) < (hysteresis / 360.0);
    }
}