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
    public static final double kTransferDefaultVel = 0.0;
    public static final double kTransferShootVel = 30.0;
    public static final double kFloorShootVel = 25.0;

    public static final double kJamDebounceTime =0.3;
    public static final double kJamReverseTime =0.2;

    public static final double kVelThreshold = 26.0;

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
        
        configs.Slot0.kP = 20.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 11.751;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.20257;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        // configs.CurrentLimits.StatorCurrentLimit = 0.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

        configs.Feedback.SensorToMechanismRatio = 30.0 / 12.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}