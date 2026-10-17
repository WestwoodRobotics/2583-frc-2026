package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransferConstants {
    public static final int kTransferId1 = 16;
    public static final CANBus kFloorCANBus = new CANBus("rio");
    public static final CANBus kTransferCANBus = new CANBus("rio");

    public static final double kFloorDefaultVel = 0.0;
    public static final double kTransferDefaultVel = 0.0;
    public static final double kTransferShootVel = 2.0;
    public static final double kTransferShootTime = 0.05;

    

    public static TalonFXConfiguration getTransferMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 30.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.2;
        configs.Slot0.kS = 5.8;
        configs.Slot0.kV = 0.148275;
        configs.Slot0.kA = 0.0;

        configs.CurrentLimits.SupplyCurrentLimit = 30.0;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;

        configs.Feedback.SensorToMechanismRatio = (40.0/18.0) * (5) * (5);
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}