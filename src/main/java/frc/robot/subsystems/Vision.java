package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pigeon2 pigeon;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;

    private final Field2d field = new Field2d();

    private boolean wasOnBump = false;
    private double landingStartTime = 0.0;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.pigeon = drivetrain.getPigeon2();

        String[] cameraNames = VisionConstants.cameraNames;
        Transform3d[] transforms = VisionConstants.robotToCamTransforms;
        AprilTagFieldLayout fieldLayout = VisionConstants.kFieldLayout;

        cameras = new PhotonCamera[cameraNames.length];
        poseEstimators = new PhotonPoseEstimator[cameraNames.length];

        for (int i = 0; i < cameraNames.length; i++) {
            cameras[i] = new PhotonCamera(cameraNames[i]);
            poseEstimators[i] = new PhotonPoseEstimator(
                fieldLayout,
                transforms[i]
            );
        }
    }

    @Override
    public void periodic() {
        updateVisionPose();
        updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        field.setRobotPose(drivetrain.getState().Pose);
        SmartDashboard.putData("Field", field);
    }

    private void updateVisionPose() {
        // Gyro Inputs
        double pitch = pigeon.getPitch().getValueAsDouble();
        double roll = pigeon.getRoll().getValueAsDouble();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();

        // Landing Snap Logic
        boolean isOnBump = Math.abs(pitch) > VisionConstants.bumpThresholdDegrees || Math.abs(roll) > VisionConstants.bumpThresholdDegrees;
        
        // Detect falling edge (was on bump, now not)
        if (wasOnBump && !isOnBump) {
            landingStartTime = Timer.getFPGATimestamp();
        }
        wasOnBump = isOnBump;

        boolean isLanding = (Timer.getFPGATimestamp() - landingStartTime) < VisionConstants.landingTimeSeconds;

        for (int i = 0; i < cameras.length; i++) {
            for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
                Optional<EstimatedRobotPose> poseOptional = poseEstimators[i].estimateCoprocMultiTagPose(result);
                if (poseOptional.isEmpty()) {
                    poseOptional = poseEstimators[i].estimateLowestAmbiguityPose(result);
                }
                if (poseOptional.isPresent()) {
                    EstimatedRobotPose pose = poseOptional.get();
                    SmartDashboard.putNumber("Vision X", pose.estimatedPose.getX());
                    SmartDashboard.putNumber("Vision Y", pose.estimatedPose.getY());
                    SmartDashboard.putNumber("Vision Rotation", pose.estimatedPose.getRotation().getAngle() * 180 /  Math.PI);
                    Matrix<N3, N1> stdDevs;

                    if (isLanding) {
                        stdDevs = VecBuilder.fill(VisionConstants.landingStdDev, VisionConstants.landingStdDev, Double.MAX_VALUE);
                    } else {
                        stdDevs = getEstimationStdDevs(pose, pitch, roll, yawRate);
                    }

                    drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdDevs);
                }
            }
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose estimatedPose, double pitch, double roll, double yawRate) {
        Matrix<N3, N1> infiniteStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        // Rejection Criteria
        if (Math.abs(pitch) > VisionConstants.bumpThresholdDegrees || Math.abs(roll) > VisionConstants.bumpThresholdDegrees) return infiniteStdDevs;
        if (Math.abs(yawRate) > VisionConstants.maxYawRateDegreesPerSec) return infiniteStdDevs;

        int numTags = estimatedPose.targetsUsed.size();
        if (numTags == 0) return infiniteStdDevs;

        if (numTags == 1 && estimatedPose.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.maxPoseAmbiguity) {
            return infiniteStdDevs;
        }

        // Confidence Calculation
        double totalArea = 0;
        for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
            totalArea += target.getArea();
        }
        if (totalArea <= 0) return infiniteStdDevs; // Prevent div by zero

        double k = (numTags > 1) ? VisionConstants.multiTagK : VisionConstants.singleTagK;
        double sigma_xy = (k / totalArea) + VisionConstants.baseSigma;
        double sigma_theta = (numTags > 1) ? VisionConstants.multiTagThetaSigma : Double.MAX_VALUE;

        return VecBuilder.fill(sigma_xy, sigma_xy, sigma_theta);
    }
}
