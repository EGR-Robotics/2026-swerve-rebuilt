package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX flywheel = new TalonFX(24, "5980");
    private final TalonFX hood = new TalonFX(23, "5980");
    
    private final TalonFX roller = new TalonFX(25, "5980");
    private final TalonFX shooterFeeder = new TalonFX(50, "5980");
    
    private static final double FLYWHEEL_GEAR_RATIO = 1.0;
    private static final double HOOD_GEAR_RATIO = 50.0;

    private static final double MIN_ANGLE = 40;
    private static final double MAX_ANGLE = 80;

    private static final double RPM_TOL = 100;
    private static final double ANGLE_TOL = 1.0;

    private static double feederSpeed = 1;
    private static final double voltageNumber = 12;

    private double goalRPM = 0;
    private double goalAngle = 60;

    private final VelocityVoltage flywheelReq = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage feederReq = new VelocityVoltage(0).withEnableFOC(true);

    private final VelocityVoltage rollerReq = new VelocityVoltage(0).withEnableFOC(true); // ADDED

    private final PositionVoltage hoodReq = new PositionVoltage(0).withEnableFOC(true);

    public Shooter() {
        flywheel.setNeutralMode(NeutralModeValue.Coast);
        hood.setNeutralMode(NeutralModeValue.Brake);
        
        shooterFeeder.setNeutralMode(NeutralModeValue.Coast);
        roller.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration flyCfg = new TalonFXConfiguration();

        flyCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        //flywheel
        flyCfg.Slot0.kP = 0.18;//tune
        flyCfg.Slot0.kI = 0.0;
        flyCfg.Slot0.kD = 0.0;
        
        flyCfg.Slot0.kV = 0.12;//tune
        flyCfg.Slot0.kS = 0.0;

        flyCfg.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

        flywheel.getConfigurator().apply(flyCfg, 0.25);

        //hood
        TalonFXConfiguration hoodCfg = new TalonFXConfiguration();
        
        hoodCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        hoodCfg.Slot0.kP = 2.0;//tune maybe
        hoodCfg.Slot0.kI = 0.0;
        hoodCfg.Slot0.kD = 0.0;
        
        hoodCfg.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        
        hood.getConfigurator().apply(hoodCfg, 0.25);

        //feeder
        TalonFXConfiguration feederCfg = new TalonFXConfiguration();
        
        feederCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        feederCfg.Slot0.kP = 0.1;//tune maybe
        feederCfg.Slot0.kI = 0.0;
        feederCfg.Slot0.kD = 0.0;
        
        shooterFeeder.getConfigurator().apply(feederCfg, 0.25);

        //rollers
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        
        rollerCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        
        rollerCfg.Slot0.kP = 0.1;//tune maybe
        rollerCfg.Slot0.kI = 0.0;
        rollerCfg.Slot0.kD = 0.0;
        
        roller.getConfigurator().apply(rollerCfg, 0.25);
    }

    public void setFlywheelRPM(double rpm) {
        goalRPM = rpm;
        double motorRPS = -(rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        flywheel.setControl(flywheelReq.withVelocity(motorRPS));
    }

    public void setFeederSpeed(double rpm){
        double motorRPS = -(rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        shooterFeeder.setControl(feederReq.withVelocity(motorRPS));
    }
    
    public void stopFeeder(){
        shooterFeeder.setControl(new VoltageOut(0));
    }

    public void setRollerRPM(double rpm){
        double motorRPS = rpm / 60.0;
        roller.setControl(rollerReq.withVelocity(motorRPS));
    }

    public void stopRoller(){
        roller.setControl(new VoltageOut(0));
    }

    public void feedAndFlywheel(double rpm){
        setFlywheelRPM(rpm);
        setFeederSpeed(rpm);
        setRollerRPM(2000); 
    }

    public void setHoodAngle(double deg) {
        deg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, deg));
        goalAngle = deg;
        double motorRot = (deg / 360.0) * HOOD_GEAR_RATIO;
        hood.setControl(hoodReq.withPosition(motorRot));
    }

    public double getFlywheelRPM() {
        double motorRPS = flywheel.getVelocity().getValueAsDouble();
        return (motorRPS / FLYWHEEL_GEAR_RATIO) * 60.0;
    }

    public double getHoodAngle() {
        double motorRot = hood.getPosition().getValueAsDouble();
        return (motorRot / HOOD_GEAR_RATIO) * 360.0;
    }

    public boolean readyToShoot() {
        return Math.abs(goalRPM - getFlywheelRPM()) < RPM_TOL &&
               Math.abs(goalAngle - getHoodAngle()) < ANGLE_TOL;
    }
}
