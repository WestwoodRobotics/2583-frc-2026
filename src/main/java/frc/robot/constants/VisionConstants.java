package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {

    public static final String[] cameraNames = {
        "frontLeft",
        "frontRight",
        "backLeft",
        "backRight"
    };

    public static final Transform3d[] robotToCamTransforms = {
        new Transform3d(
            new Translation3d(0.298069, -0.0019304, 0.317627),
            new Rotation3d(0.0, Math.toRadians(12.87), Math.toRadians(25.0))
        ),
        new Transform3d(
            new Translation3d(0.298069, -0.0997204, 0.317627),
            new Rotation3d(0.0, Math.toRadians(12.87), Math.toRadians(-25.0))
        ),
        new Transform3d(
            new Translation3d(0.1518158, 0.377825, 0.2233422),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(7.067), Math.toRadians(90 + 4.981))
        ),
        new Transform3d(
            new Translation3d(0.1518158, -0.377825, 0.2233422),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(7.067), Math.toRadians(-90 - 4.981))
        ),
    };

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double bumpThresholdDegrees = 10.0;
    public static final double landingTimeSeconds = 0.3;
    public static final double landingStdDev = 0.1;
    public static final double maxYawRateDegreesPerSec = 270.0;
    public static final double maxPoseAmbiguity = 0.2;
    public static final double multiTagK = 0.2;
    public static final double singleTagK = 0.9;
    public static final double baseSigma = 0.2;
    public static final double multiTagThetaSigma = 8.0;
    public static final int maxMeasurementsToApply = 2;
}