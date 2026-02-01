package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimShooter extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest;

    public AimShooter(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle request, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.driveRequest = request;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return;
        }
        var alliance = allianceOpt.get();
        boolean isBlue = (alliance == Alliance.Blue);

        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d shooterPose = robotPose.transformBy(SwerveConstants.shooterToRobot);
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

        Rotation2d targetHeading = targetLocation.minus(shooterPose.getTranslation())
            .getAngle()
            .minus(SwerveConstants.shooterToRobot.getRotation());
        
        double[] drives = CommandSwerveDrivetrain.joyStickPolar(controller, 2);

        drivetrain.setControl(driveRequest
            .withVelocityX(drives[0])
            .withVelocityY(drives[1])
            .withTargetDirection(targetHeading));
    }
}