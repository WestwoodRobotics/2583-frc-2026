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
            new Translation3d(-0.05943, 0.34998, 0.48351),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(78))
        ),
        new Transform3d(
            new Translation3d(-0.05943, -0.34998, 0.48351),
            new Rotation3d(0.0, Math.toRadians(0), Math.toRadians(-78))
        ),
        new Transform3d(
            new Translation3d(-0.31573, 0.02052, 0.51911),
            new Rotation3d(Math.toRadians(.0), Math.toRadians(0), Math.toRadians(140))
        ),
        new Transform3d(
            new Translation3d(-0.31573, -0.02052, 0.47039),
            new Rotation3d(Math.toRadians(0.0), Math.toRadians(0), Math.toRadians(-140))
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
    public static final double baseSigma = 0.08;
    public static final double multiTagThetaSigma = 8.0;
    public static final int maxMeasurementsToApply = 2;
}