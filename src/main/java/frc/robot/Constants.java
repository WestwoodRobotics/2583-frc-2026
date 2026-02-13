package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.io.IOException;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

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

    public static final class VisionConstants {
        public static final String[] cameraNames = {
            "frontLeft",
            "frontRight"
        };

        public static final Transform3d[] robotToCamTransforms = {
            new Transform3d(
                new Translation3d(0.26430478, 0.3150616, 0.1874266),
                new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(-41.641))
            ),
            new Transform3d(
                new Translation3d(0.26430478, -0.3150616, 0.1874266),
                new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(41.641))
            ),
        };

        public static final AprilTagFieldLayout kFieldLayout;
        
        static {
            try {
                kFieldLayout = new AprilTagFieldLayout("/home/lvuser/deploy/2026-rebuilt-andymark.json");
            } catch (IOException e) {
                throw new RuntimeException("Failed to load AprilTag field layout", e);
            }
        }

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
}
