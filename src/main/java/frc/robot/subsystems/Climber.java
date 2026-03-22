package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final TalonFX leftClimber = new TalonFX(40, "5980");
    private final TalonFX rightClimber = new TalonFX(41, "5980");

    private static double direction = 1.0;

    private static final double climbSpeed = 0.8;
    private static final double voltageNumber = 12;


    public Climber() {
    }

    /** Run only the left climber */
    public void climbLeftJoystick(double axis) {
        leftClimber.setControl(new VoltageOut(climbSpeed * voltageNumber * axis));
        rightClimber.setControl(new VoltageOut(0));
        setBrakeMode(false);
    }

    public void climbLeft() {
        leftClimber.setControl(new VoltageOut(climbSpeed * voltageNumber * direction));
        rightClimber.setControl(new VoltageOut(0));
        setBrakeMode(false);
    }

    //for auto
    public void autoClimbUp(double triggerVal) {
        leftClimber.setControl(new VoltageOut(-triggerVal * climbSpeed * voltageNumber));
    }

    public void autoClimbDown() {
        leftClimber.setControl(new VoltageOut(climbSpeed * voltageNumber));
    }

    // /** Run only the right climber */
    // public void climbRight() {
    //     rightClimber.setControl(new VoltageOut(climbSpeed * voltageNumber * direction));
    //     leftClimber.setControl(new VoltageOut(0));
    //     setBrakeMode(false);
    // }

    // /** Run both climbers */
    // public void climbBoth() {
    //     leftClimber.setControl(new VoltageOut(climbSpeed * voltageNumber * direction));
    //     rightClimber.setControl(new VoltageOut(climbSpeed * voltageNumber * direction));
    //     setBrakeMode(false);
    // }

    /** Stop both climbers */
    public void stop() {
        leftClimber.setControl(new VoltageOut(0));
        //rightClimber.setControl(new VoltageOut(0));
        setBrakeMode(true);
    }

    /** Toggle brake mode */
    public void toggleDirection() { 
        if (direction >= 0) {
            direction = -1.0;
        } else {
            direction = 1.0;
        }
    }

    /** Apply brake or coast mode */
    private void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        leftClimber.setNeutralMode(mode);
        rightClimber.setNeutralMode(mode);
    }
}
