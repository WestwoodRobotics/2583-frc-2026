package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;

public class GetTargetLocation {

    /**
     * Number of iterations to run when converging on a moving target solution. More iterations
     * increase accuracy but also latency.
     */
    private static final int kTargetingIterations = 4;

    private static double mLastTimestamp = -1.0;
    private static Translation2d m_targetLocation = null;

    public static Translation2d getTargetLocation(Pose2d robotPose, ChassisSpeeds currentSpeeds) {
        // Cache result to avoid recalculating multiple times per loop cycle (approx 20ms)
        double currentTimestamp = Timer.getFPGATimestamp();
        if (Math.abs(currentTimestamp - mLastTimestamp) < 0.005 && m_targetLocation != null) {
            return m_targetLocation;
        }

        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return null;
        }
        var alliance = allianceOpt.get();
        boolean isBlue = (alliance == Alliance.Blue);

        // Determine if we are in the alliance's shooting zone based on the robot's X position.
        boolean inZone;
        if (isBlue) {
            inZone = robotPose.getX() < SwerveConstants.allianceZoneWidth;
        } else {
            inZone = robotPose.getX() > (SwerveConstants.fieldWidth - SwerveConstants.allianceZoneWidth);
        }

        if (inZone) {
            // Lock to appropriate hub
            m_targetLocation = isBlue ? SwerveConstants.blueHub : SwerveConstants.redHub;
        } else {
            // Lock to alliance corner
            boolean isTop = robotPose.getY() >= (SwerveConstants.fieldLength / 2.0);
            
            double targetX = isBlue ? SwerveConstants.bluePassX : SwerveConstants.redPassX;
            double targetY = isTop ? SwerveConstants.upperPassY : SwerveConstants.lowerPassY;
            
            m_targetLocation = new Translation2d(targetX, targetY);
        }

        m_targetLocation = adjustMovingTarget(robotPose, currentSpeeds, m_targetLocation);
        mLastTimestamp = currentTimestamp;
        return m_targetLocation;
    }

    public static Translation2d adjustMovingTarget(Pose2d robotPose, ChassisSpeeds currentSpeeds, Translation2d realTargetPos) {
        // Convert field-relative chassis speeds to a Translation2d vector
        Translation2d robotVelocity = new Translation2d(
            currentSpeeds.vxMetersPerSecond, 
            currentSpeeds.vyMetersPerSecond
        ).rotateBy(robotPose.getRotation());

        // Initial guess: the virtual target is just the real target
        Translation2d virtualTarget = realTargetPos;
        Translation2d robotMovement;

        // Iterate 3 times to converge on the correct virtual target
        for (int i = 0; i < kTargetingIterations; i++) {
            // 1. Find distance from robot to the CURRENT virtual target
            double distance = robotPose.getTranslation().getDistance(virtualTarget);

            // 2. Look up the Time of Flight for that distance
            // NOTE: This assumes kTOFMap contains an entry for every possible distance.
            // If 'distance' is not in the map, this will cause a NullPointerException.
            // Consider using interpolation or a method that handles out-of-range values.
            double timeOfFlight = ShooterConstants.kDistanceToTOF.get(distance);

            // 3. Calculate how much the robot will move during that time
            robotMovement = robotVelocity.times(timeOfFlight);

            // 4. Shift the target in the OPPOSITE direction of the robot's movement
            virtualTarget = realTargetPos.minus(robotMovement);
        }
        return virtualTarget;
    }
}
