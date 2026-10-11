package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TransferConstants {
    public static final int kFloorId = 999;
    public static final int kTransferId1 = 999;
    public static final int kTransferId2 = 999;
    public static final CANBus kFloorCANBus = new CANBus("SwerveCAN");
    public static final CANBus kTransferCANBus = new CANBus("rio");

    public static final double kFloorDefaultVel = 0.0;
    public static final double kTransferDefaultVel = 0.0;
    public static final double kTransferShootVel = 50.0;
    public static final double kFloorShootVel = 22.0;
    public static final double kTransferShootTime = 0.05;

    

    public static TalonFXConfiguration getTransferMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 25.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.1;
        configs.Slot0.kS = 10.5;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0971466619;

        configs.CurrentLimits.SupplyCurrentLimit = 30.0;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        configs.Feedback.SensorToMechanismRatio = 18.0/40.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}