package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {

    public static final class LEDConstants {
        public static final int candleId = 30;
        public static final CANBus canBus = new CANBus("SwerveCAN");
        public static final int endIndex = 26;
    }

    public static final class SwerveConstants {

        public static final double fieldWidth = 16.540988;
        public static final double fieldLength = 8.069326;
        public static final double allianceZoneWidth = 4.625594;

        public static final Transform2d shooterToRobot = new Transform2d(
            new Translation2d(0.0, 0.0),
            new Rotation2d(Math.toRadians(0))
        );

        public static final Translation2d blueHub = new Translation2d(allianceZoneWidth, fieldLength / 2);
        public static final Translation2d redHub = new Translation2d(fieldWidth - allianceZoneWidth, fieldLength / 2);

        public static final Pose2d leftTower = new Pose2d(2.0, 5.0, new Rotation2d());
        public static final double alignMaxVel = 1.0;
        public static final double alignMaxAccel = 1.0;
        public static final double alignMaxOmega = 0.75;
        public static final double alignMaxAlpha = 0.75;

        public static final double aimKp = 4.0;
        public static final double aimKi = 0.0;
        public static final double aimKd = 0.1;
    }

    public static final class TransferConstants {
        public static final int kFloorId1 = 22;
        public static final int kFloorId2 = 29;
        public static final int kTransferId = 23;
        public static final String kCANBus = "SwerveCAN";

        public static final double kFloorDefaultVel = 0.0;
        public static final double kFloorIntakeVel = 0.0;
        public static final double kTransferDefaultVel = 0.0;
        public static final double kTransferShootVel = 0.0;
        public static final double kFloorShootVel = 0.0;

        public static TalonFXConfiguration getFloorMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;

            configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

            // configs.CurrentLimits.StatorCurrentLimit = 0.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getTransferMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;

            configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

            // configs.CurrentLimits.StatorCurrentLimit = 0.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }
    }
}
