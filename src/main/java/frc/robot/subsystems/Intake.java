package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeRoller = new TalonFX(27, "5980");
    private final TalonFX intakePivot = new TalonFX(26, "5980");
    private final CANcoder pivotEncoder = new CANcoder(8, "5980"); // or 9 if needed

    private final PositionVoltage pivotRequest = new PositionVoltage(0);

    private static final double intakeSpeed = 0.9; // TODO: Change the speeds.
    private static final double lowerIntakeSpeed = 0.9;
    private static final double raiseIntakeSpeed = -1.0;
    private static final double voltageNumber = 12;
    private static final double intakeRotation = 10; // degrees
    private static final double hysteresis = 1.0;    // degrees

    private final double pivotUpPosition;
    private final double pivotDownPosition;

    public Intake() {

        double absolute = pivotEncoder.getAbsolutePosition().getValueAsDouble(); 
        // absolute is 0–1 rotations

        pivotUpPosition = absolute; 
        pivotDownPosition = pivotUpPosition + (intakeRotation / 360.0);
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
        if (this.isIntakeDown()) return;

        intakePivot.setControl(pivotRequest.withPosition(pivotDownPosition));
    }

    /** Raise intake */
    public void raiseIntake() {
        if (this.isIntakeUp()) return;

        intakePivot.setControl(pivotRequest.withPosition(pivotUpPosition));
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

    private double getPivotPosition() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble();
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
