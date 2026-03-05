package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.utils.ShotParam;

import java.util.TreeMap;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {

    public static final class LEDConstants {
        public static final int candleId = 50;
        public static final CANBus canBus = new CANBus("SwerveCAN");
        public static final int endIndex = 26;

        public static final double kMaxHeadingError = 2.0;
    }

    public static final class SwerveConstants {

        public static final double fieldWidth = 16.513048;
        public static final double fieldLength = 8.042656;
        public static final double allianceZoneWidth = 4.611624;

        public static final Transform2d robotToShooter = new Transform2d(
            new Translation2d(0.1344549, 0.0),
            new Rotation2d(Math.toRadians(0))
        );

        public static final Translation2d blueHub = new Translation2d(allianceZoneWidth, fieldLength / 2);
        public static final Translation2d redHub = new Translation2d(fieldWidth - allianceZoneWidth, fieldLength / 2);

        public static final double upperCornerY = 7.350;
        public static final double lowerCornerY = fieldLength - upperCornerY;
        public static final double blueCornerX = 4;
        public static final double redCornerX = fieldWidth - blueCornerX;

        public static final double upperBlueCornerAngle = Math.toRadians(-79.58837132);
        public static final double lowerBlueCornerAngle = Math.toRadians(79.58837132);
        public static final double upperRedCornerAngle = Math.toRadians(79.58837132 - 180);
        public static final double lowerRedCornerAngle = Math.toRadians(-79.58837132 + 180);

        public static final double alignMaxVel = 3;
        public static final double alignMaxAccel = 3;
        public static final double alignMaxOmega = 1.5;
        public static final double alignMaxAlpha = 2;

        public static final double bluePassX = 2.5;
        public static final double redPassX = fieldWidth - bluePassX;
        public static final double lowerPassY = 1.5;
        public static final double upperPassY = fieldLength - lowerPassY;

        public static final double aimKp = 11.0;
        public static final double aimKi = 0.0;
        public static final double aimKd = 0.;
    }

    public static final class IntakeConstants {
        public static final int kPivotMotorId = 20;
        public static final int kRollerMotorId = 21;
        public static final CANBus kCANBus = new CANBus("SwerveCAN");

        public static final double kPivotOffset = 0.330810546875;

        public static final double pivotIn = 0.330810546875;
        public static final double pivotPartial = 0.00;
        public static final double pivotOut = -0.17;

        public static final double rollerNeutralVel = 0.0;
        public static final double rollerIntakingVel = 35.0;
        public static final double rollerShootingVel = 25.0;

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
            
            configs.MotionMagic.MotionMagicAcceleration = 7;
            configs.MotionMagic.MotionMagicCruiseVelocity = 2;

            // configs.CurrentLimits.StatorCurrentLimit = 80.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 60.0;

            configs.Feedback.SensorToMechanismRatio = (70.0 / 11.0) * (40.0 / 12.0);
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getRollerConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 20.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 17.774;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.16294;

            configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

            // configs.CurrentLimits.StatorCurrentLimit = 80.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 60.0;

            configs.Feedback.SensorToMechanismRatio = (2.0/1.0 * 30.0/24.0);

            return configs;
        }
    }

    public static final class ShooterConstants {
        public static final int kHoodMotorId = 24;
        public static final int kTopLeftFlywheelId = 27;
        public static final int kTopRightFlywheelId = 25;
        public static final int kBottomLeftFlywheelId = 28;
        public static final int kBottomRightFlywheelId = 26;
        public static final CANBus kCANBus = new CANBus("SwerveCAN");

        public static final double kMaxFlywheelRPS = 80.0;

        public static final double kPosAtMinAngle = 0.091552734375;
        public static final double kPosAtMaxAngle = 0.0;
        public static final double kMinAngle = 54.96;
        public static final double kMaxAngle = 90.0;
        public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

        public static final double kManualHoodVolts = 1.0;
        public static final double kManualFlywheelInc = 2.5;

        public static final TreeMap<Double, ShotParam> kDistanceToShotParam = new TreeMap<>();

        public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

        static {
            kDistanceToShotParam.put(0.0, new ShotParam(90.0, 55.0));
            kDistanceToShotParam.put(0.8763, new ShotParam(83.0, 55.0));
            kDistanceToShotParam.put(1.0287, new ShotParam(78.0, 45.0));
            kDistanceToShotParam.put(1.1811, new ShotParam(75.44, 45.0));
            kDistanceToShotParam.put(1.4351, new ShotParam(70.689, 45.0));
            kDistanceToShotParam.put(1.6891, new ShotParam(65.55, 45.0));
            kDistanceToShotParam.put(1.9431, new ShotParam(63.99, 45.0));
            kDistanceToShotParam.put(2.4003, new ShotParam(54.96, 45.0));
            kDistanceToShotParam.put(2.9083, new ShotParam(54.96, 50.0));
            kDistanceToShotParam.put(3.4163, new ShotParam(54.96, 52.5));
            kDistanceToShotParam.put(3.9243, new ShotParam(54.96, 57.5));
            kDistanceToShotParam.put(20.0, new ShotParam(54.96, kMaxFlywheelRPS));

            kDistanceToTOF.put(0.0, 0.0);
            kDistanceToTOF.put(999.0, 0.0);
        }

        public static TalonFXConfiguration getHoodMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;
            configs.Slot0.kG = 0.0;

            // configs.CurrentLimits.StatorCurrentLimit = 0.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = (52.0 / 11.0) * (110.0 / 7.0);
            
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getFlywheelMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 10.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 2.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.2;

            // configs.CurrentLimits.StatorCurrentLimit = 0.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

            configs.Feedback.SensorToMechanismRatio = 42/16 * 15/33;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            
            return configs;
        }
    }
    
    public static final class VisionConstants {

        public static final String[] cameraNames = {
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight"
        };

        public static final Transform3d[] robotToCamTransforms = {
            new Transform3d(
                new Translation3d(0.26430478, 0.3150616, 0.1874266),
                new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(41.641))
            ),
            new Transform3d(
                new Translation3d(0.26430478, -0.3150616, 0.1874266),
                new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(-41.641))
            ),
            new Transform3d(
                new Translation3d(-0.129286, 0.352552, 0.240792),
                new Rotation3d(0.0, Math.toRadians(27.08), Math.toRadians(180 - 48.36))
            ),
            new Transform3d(
                new Translation3d(-0.129286, -0.352552, 0.240792),
                new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(180 + 48.36))
            )
        };

        public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double bumpThresholdDegrees = 8.0;
        public static final double landingTimeSeconds = 0.25;
        public static final double landingStdDev = 0.1;
        public static final double maxYawRateDegreesPerSec = 720.0;
        public static final double maxPoseAmbiguity = 0.2;
        public static final double multiTagK = 0.2;
        public static final double singleTagK = 0.8;
        public static final double baseSigma = 0.05;
        public static final double multiTagThetaSigma = 2.0;
    }

    public static final class TransferConstants {
        public static final int kFloorId1 = 22;
        public static final int kFloorId2 = 29;
        public static final int kTransferId1 = 23;
        public static final int kTransferId2 = 30;
        public static final String kCANBus = "SwerveCAN";

        public static final double kFloorDefaultVel = 0.0;
        public static final double kFloorIntakeVel = 0.0;
        public static final double kTransferDefaultVel = 0.0;
        public static final double kTransferShootVel = 30.0;
        public static final double kFloorShootVel = 20.0;

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

            configs.Feedback.SensorToMechanismRatio = 47.0/11.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getTransferMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 40.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 11.751;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.20257;

            // configs.CurrentLimits.StatorCurrentLimit = 0.0;
            // configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 33.0/15.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }
    }
}
