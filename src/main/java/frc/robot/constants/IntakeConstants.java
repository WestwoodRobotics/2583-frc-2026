package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static final int kPivotMotorOneId = 14;
    public static final int kPivotMotorTwoId = 15;

    public static final int kRollerMotorId = 21;
    public static final int kRollerMotorTwoId = 22;

    public static final CANBus kCANBus = new CANBus("rio");

    public static final double kPivotOffset = 0.15; //when intake in

    public static final double kPivotIn = kPivotOffset;
    public static final double kPivotPartial = 0.00;
    public static final double kPivotOut = -0.15; //when intake out, should be around 0.4 away
    public static final double kPivotShoot = -0.06; //intake out + 0.2
    public static final double kPivotUp = -0.04; //
    public static final double kPivotDepot = -0.03596875;
    public static final double kResetPivotPos = -0.066162109375;

    public static final double kRollerNeutralVel = 0.0;
    public static final double kRollerIntakingVel = 20.0;
    public static final double kRollerShootingVel = 0.0;

    public static final double kJammedThreshold = 0.03;

    public static TalonFXConfiguration getPivotConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 50.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 5.0;    
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kG = 0.0;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        configs.MotionMagic.MotionMagicAcceleration = 9.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 3.0;

        configs.CurrentLimits.SupplyCurrentLimit = 45.0;

        configs.Feedback.SensorToMechanismRatio = (100.0/15.0) * (60.0/46.0) * (50.0/30.0) * (38.0/12.0);
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getRollerConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 15.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 10.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;

        // configs.CurrentLimits.SupplyCurrentLimit = 50.0;
        
        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        configs.Feedback.SensorToMechanismRatio = 1.5;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return configs;
    }
}