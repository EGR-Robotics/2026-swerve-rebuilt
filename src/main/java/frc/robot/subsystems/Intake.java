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

    private static final double intakeSpeed = 0.9;
    private static final double lowerIntakeSpeed = 0.9;
    private static final double raiseIntakeSpeed = -1.0;
    private static final double voltageNumber = 12;
    private static final double intakeRotation = 210; // degrees
    private static final double hysteresis = 1.0;    // degrees

    // HARD‑CODED OFFSET (your value goes here)
    private static final double pivotUpOffset = 0.3842; // <<< replace with your measured absolute

    private final double pivotUpPosition;
    private final double pivotDownPosition;

    public Intake() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        cfg.Feedback.FeedbackRemoteSensorID = 9; 
        
        cfg.Slot0.kP = 6.0;  
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        
        intakePivot.getConfigurator().apply(cfg);

        pivotUpPosition = pivotUpOffset;
        pivotDownPosition = pivotUpOffset + (intakeRotation / 360.0);

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
        if (this.isIntakeDown()) return;
        intakePivot.setControl(pivotRequest.withPosition(pivotDownPosition));
    }

    public void raiseIntake() {
        if (this.isIntakeUp()) return;
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
        double abs = pivotEncoder.getAbsolutePosition().getValueAsDouble();

        double corrected = abs - pivotUpOffset;
        if (corrected < 0) corrected += 1.0;
        if (corrected > 1) corrected -= 1.0;

        return corrected + pivotUpOffset;
    }

    private boolean isIntakeUp() {
        double pos = getPivotPosition();
        return Math.abs(pos - pivotUpPosition) < (hysteresis / 360.0);
    }

    private boolean isIntakeDown() {
        double pos = getPivotPosition();
        return Math.abs(pos - pivotDownPosition) < (hysteresis / 360.0);
    }
}
