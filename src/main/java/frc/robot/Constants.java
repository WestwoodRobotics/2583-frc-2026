package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {

    public static final class LEDConstants {
        public static final int candleId = 30;
        public static final String canBus = "CANivore";
        public static final int endIndex = 26;
    }

    public static final class TransferConstants {
        public static final int kFloorId = 1;
        public static final int kTransferId = 2;
        public static final String kCANBus = "CANivore";

        public static final double kFloorDefaultVel = 0.0;
        public static final double kFloorIntakeVel = 0.0;
        public static final double kTransferDefaultVel = 0.0;
        public static final double kTransferShootVel = 0.0;
        public static final double kFloorShootVel = 0.0;

        public static TalonFXConfiguration getVelocityMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;

            configs.CurrentLimits.StatorCurrentLimit = 0.0;
            configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.MotionMagic.MotionMagicAcceleration = 0.0;
            configs.MotionMagic.MotionMagicJerk = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }
    }
}
