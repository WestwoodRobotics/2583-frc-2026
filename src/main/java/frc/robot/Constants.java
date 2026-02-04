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

        public static final double fieldWidth = 16.513048;
        public static final double fieldLength = 8.042656;
        public static final double allianceZoneWidth = 4.611624;

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

    public static final class ShooterConstants {
        public static final int kHoodMotorId = 1;
        public static final int kTopLeftFlywheelId = 2;
        public static final int kTopRightFlywheelId = 3;
        public static final int kBottomLeftFlywheelId = 4;
        public static final int kBottomRightFlywheelId = 5;
        public static final CANBus kCANBus = new CANBus("rio");

        public static final double hoodInPos = 0.0;

        public static final double shootingVel = 0.0;

        public static final double posAtMinAngle = 0.0;
        public static final double posAtMaxAngle = 0.0;
        public static final double minAngle = 0.0;
        public static final double maxAngle = 0.0;
        public static final double perDegree = (posAtMaxAngle - posAtMinAngle) / (maxAngle - minAngle);

        public static TalonFXConfiguration getHoodMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;
            configs.Slot0.kG = 0.0;
            
            configs.MotionMagic.MotionMagicExpo_kV = 0.0;
            configs.MotionMagic.MotionMagicExpo_kA = 0.0;

            configs.CurrentLimits.StatorCurrentLimit = 0.0;
            configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getFlywheelMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;

            configs.MotionMagic.MotionMagicAcceleration = 0.0;

            configs.CurrentLimits.StatorCurrentLimit = 0.0;
            configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }
    }
}
