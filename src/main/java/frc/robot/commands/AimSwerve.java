package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.GetTargetLocation;

public class AimSwerve extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest;

    public AimSwerve(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle request, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.driveRequest = request;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d shooterPose = robotPose.transformBy(SwerveConstants.shooterToRobot);

        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose);

        double[] drives = CommandSwerveDrivetrain.joyStickPolar(controller, 2);

        if (targetLocation == null) {
            drivetrain.setControl(driveRequest
                .withVelocityX(drives[0])
                .withVelocityY(drives[1])
                .withTargetDirection(robotPose.getRotation()));
            return;
        }

        Rotation2d targetHeading = targetLocation.minus(shooterPose.getTranslation())
            .getAngle()
            .minus(SwerveConstants.shooterToRobot.getRotation());

        drivetrain.setControl(driveRequest
            .withVelocityX(drives[0])
            .withVelocityY(drives[1])
            .withTargetDirection(targetHeading));
    }
}