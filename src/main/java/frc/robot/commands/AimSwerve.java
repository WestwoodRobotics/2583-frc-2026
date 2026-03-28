package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.GetTargetLocation;

public class AimSwerve extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;

    private final SwerveRequest.FieldCentricFacingAngle driveRequest;
    private final double[] driverInputs = new double[3];

    public AimSwerve(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle request, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.driveRequest = request;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;
        // Pose2d shooterPose = robotPose.plus(SwerveConstants.robotToShooter);

        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose, drivetrain.getState().Speeds);

        if (DriverStation.isTeleop()) {
            CommandSwerveDrivetrain.joyStickPolar(driverInputs, controller, 2);
        } else {
            driverInputs[0] = 0;
            driverInputs[1] = 0;
            driverInputs[2] = 0;
        }

        if (targetLocation == null) {
            drivetrain.setControl(driveRequest
                .withVelocityX(driverInputs[0])
                .withVelocityY(driverInputs[1])
                .withTargetDirection(robotPose.getRotation()));
            return;
        }

        Rotation2d targetHeading = targetLocation.minus(robotPose.getTranslation())
            .getAngle();

/*         double HubHeading = targetHeading.getRadians() + (AdjustShooter.headingCorrection);
 */     
        //if we are below 0.5 m/s, add our heading correction as normal
        double HubHeading = targetHeading.getRadians() - (AdjustShooter.headingCorrection * SwerveConstants.kCompensationP);
        
        //if we are above 0.5 m/s, add our heading correction as normal and an extra compensation
        if(Math.abs(AdjustShooter.perpendicularVel) > SwerveConstants.kSOTMVel){

            if(AdjustShooter.headingCorrection > 0){
                HubHeading += (SwerveConstants.kCompensationFF);
            } else{
                HubHeading -= (SwerveConstants.kCompensationFF);
            }
        }
        

        Rotation2d correctedHeading = new Rotation2d(HubHeading);

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            correctedHeading = correctedHeading.minus(new Rotation2d(Math.toRadians(180.0)));
        }

        drivetrain.setControl(driveRequest
            .withVelocityX(driverInputs[0])
            .withVelocityY(driverInputs[1])
            .withTargetDirection(correctedHeading));
    }
}