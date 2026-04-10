package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static final int kPivotMotorId = 20;
    public static final int kRollerMotorId = 21;
    public static final CANBus kCANBus = new CANBus("SwerveCAN");

    public static final double kPivotOffset = 0.33;

    public static final double kPivotIn = kPivotOffset;
    public static final double kPivotPartial = 0.00;
    public static final double kPivotOut = -0.11;
    public static final double kPivotShoot = 0.16;
    public static final double kResetPivotPos = -0.066162109375;

    public static final double kRollerNeutralVel = 0.0;
    public static final double kRollerIntakingVel = 35.0;
    public static final double kRollerShootingVel = 15.0;

    public static TalonFXConfiguration getPivotConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 240.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 60.0;
        configs.Slot0.kS = 5.0;    
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 2.0;
        configs.Slot0.kG = 17.0;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        configs.MotionMagic.MotionMagicAcceleration = 7.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 2.0;

        // configs.CurrentLimits.StatorCurrentLimit = 80.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 60.0;

        configs.Feedback.SensorToMechanismRatio = (70.0 / 11.0) * (40.0 / 12.0);
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getRollerConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 5.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 16.7;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.28155307749;

        // configs.CurrentLimits.StatorCurrentLimit = 80.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 50.0;

        configs.Feedback.SensorToMechanismRatio = (29.0 / 12.0);
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return configs;
    }
}