package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransferConstants {
    public static final int kFloorId = 22;
    public static final int kTransferId1 = 23;
    public static final int kTransferId2 = 24;
    public static final CANBus kFloorCANBus = new CANBus("SwerveCAN");
    public static final CANBus kTransferCANBus = new CANBus("rio");

    public static final double kFloorDefaultVel = 0.0;
    public static final double kTransferDefaultVel = 0.0;
    public static final double kTransferShootVel = 39.0;
    public static final double kFloorShootVel = 22.0;

    public static TalonFXConfiguration getFloorMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 15.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 4.8;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.245143636466;

        // configs.CurrentLimits.StatorCurrentLimit = 0.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

        configs.Feedback.SensorToMechanismRatio = 42.0 / 12.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getTransferMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 5.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 3.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.248069211307;

        // configs.CurrentLimits.StatorCurrentLimit = 0.0;
        // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

        configs.Feedback.SensorToMechanismRatio = 2.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}