package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feed extends SubsystemBase{
    
    private final TalonFX roller = new TalonFX(25, "5980");
    private final TalonFX shooterFeeder = new TalonFX(50, "5980");
    
    private final VelocityVoltage feederReq = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage rollerReq = new VelocityVoltage(0).withEnableFOC(true);

    public Feed(){
        shooterFeeder.setNeutralMode(NeutralModeValue.Coast);
        roller.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration feederCfg = new TalonFXConfiguration();
        feederCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        feederCfg.Slot0.kP = 0.1;
        feederCfg.Slot0.kI = 0.0;
        feederCfg.Slot0.kD = 0.0;
        shooterFeeder.getConfigurator().apply(feederCfg, 0.25);

        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        rollerCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerCfg.Slot0.kP = 0.1;
        rollerCfg.Slot0.kI = 0.0;
        rollerCfg.Slot0.kD = 0.0;
        roller.getConfigurator().apply(rollerCfg, 0.25);
    }

    public void setFeederSpeed(double rpm){
        double motorRPS = -(rpm / 60.0);
        shooterFeeder.setControl(feederReq.withVelocity(motorRPS));
    }
    
    public void stopFeeder(){
        shooterFeeder.setControl(new VoltageOut(0));
    }

    public void setRollerRPM(double rpm){
        double motorRPS = -rpm / 60.0;
        roller.setControl(rollerReq.withVelocity(motorRPS));
    }

    public void stopRoller(){
        roller.setControl(new VoltageOut(0));
    }

    public void feedFuel(double rpm){
        setRollerRPM(rpm);
        setFeederSpeed(rpm);
    }

    public void reverseRoller(double rpm){
        setRollerRPM(-rpm);
    }

     public void stopFeed(double rpm){
        stopRoller();
        stopFeeder();
    }
}
