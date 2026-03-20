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
        // "backLeft",
        // "backRight"
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
        // new Transform3d(
        //     new Translation3d(-0.129286, 0.352552, 0.240792),
        //     new Rotation3d(0.0, Math.toRadians(27.08), Math.toRadians(180 - 48.36))
        // ),
        // new Transform3d(
        //     new Translation3d(-0.129286, -0.352552, 0.240792),
        //     new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(180 + 48.36))
        // )
    };

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double bumpThresholdDegrees = 10.0;
    public static final double landingTimeSeconds = 0.4;
    public static final double landingStdDev = 0.1;
    public static final double maxYawRateDegreesPerSec = 360.0;
    public static final double maxPoseAmbiguity = 0.2;
    public static final double multiTagK = 0.2;
    public static final double singleTagK = 0.8;
    public static final double baseSigma = 0.1;
    public static final double multiTagThetaSigma = 2.0;
    public static final int maxMeasurementsToApply = 2;
}