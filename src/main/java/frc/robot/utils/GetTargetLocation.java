package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class GetTargetLocation {

    public static Translation2d getTargetLocation(Pose2d robotPose) {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return null;
        }
        var alliance = allianceOpt.get();
        boolean isBlue = (alliance == Alliance.Blue);

        Translation2d targetLocation;

        // Determine if we are in the alliance zone
        // fieldWidth is the long dimension (X), fieldLength is the short dimension (Y) based on Constants usage
        boolean inZone;
        if (isBlue) {
            inZone = robotPose.getX() < SwerveConstants.allianceZoneWidth;
        } else {
            inZone = robotPose.getX() > (SwerveConstants.fieldWidth - SwerveConstants.allianceZoneWidth);
        }

        if (inZone) {
            // Lock to appropriate hub
            targetLocation = isBlue ? SwerveConstants.blueHub : SwerveConstants.redHub;
        } else {
            // Lock to alliance corner
            boolean isTop = robotPose.getY() >= (SwerveConstants.fieldLength / 2.0);
            
            double targetX = isBlue ? 0.0 : SwerveConstants.fieldWidth;
            double targetY = isTop ? SwerveConstants.fieldLength : 0.0;
            
            targetLocation = new Translation2d(targetX, targetY);
        }
        return targetLocation;
    }
}
