package frc.robot.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransferConstants {
    public static final int kFloorId1 = 22;
    public static final int kTransferId1 = 23;
    public static final int kTransferId2 = 30;
    public static final String kCANBus = "SwerveCAN";

    public static final double kFloorDefaultVel = 0.0;
    public static final double kFloorIntakeVel = 0.0;
    public static final double kTransferDefaultVel = -5.0;
    public static final double kTransferShootVel = 30.0;
    public static final double kFloorShootVel = 5.0;

    public static final double kJamThreshold = 0.5;
    public static final double kJamTimeSecs = 0.25;
    public static final double kReverseTimeSecs = 0.5;

    public static TalonFXConfiguration getFloorMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 15.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 10.365;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.21273;

        // configs.CurrentLimits.StatorCurrentLimit = 0.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

        configs.Feedback.SensorToMechanismRatio = 42.0 / 12.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getTransferMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 10.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 11.751;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.20257;

        // configs.CurrentLimits.StatorCurrentLimit = 0.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

        configs.Feedback.SensorToMechanismRatio = 30.0 / 12.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}