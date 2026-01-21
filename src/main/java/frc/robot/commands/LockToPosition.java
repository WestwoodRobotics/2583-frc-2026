package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockToPosition extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier translationX;
    private final DoubleSupplier translationY;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public LockToPosition(CommandSwerveDrivetrain drivetrain, DoubleSupplier translationX, DoubleSupplier translationY) {
        this.drivetrain = drivetrain;
        this.translationX = translationX;
        this.translationY = translationY;
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

        drivetrain.setControl(driveRequest
            .withVelocityX(translationX.getAsDouble())
            .withVelocityY(translationY.getAsDouble())
            .withTargetDirection(targetHeading));
    }
}