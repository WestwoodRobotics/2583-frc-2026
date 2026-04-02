package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
    private final SwerveRequest.SwerveDriveBrake brakeRequest;
    private final double[] driverInputs = new double[3];

    private final BooleanSubscriber m_canShootSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("CanShoot")
        .subscribe(true);
    
    private final Timer brakeTimer = new Timer();
    private boolean isAiming = false;

    public AimSwerve(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle request, SwerveRequest.SwerveDriveBrake brake, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.driveRequest = request;
        this.brakeRequest = brake;
        addRequirements(drivetrain);
        brakeTimer.start();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        boolean canShoot = m_canShootSub.get();
        boolean barelyMoving = velocity < SwerveConstants.kMinAimSpeed;

        if (DriverStation.isTeleop()) {
            CommandSwerveDrivetrain.joyStickPolar(driverInputs, controller, 2);
        } else {
            driverInputs[0] = 0;
            driverInputs[1] = 0;
            driverInputs[2] = 0;
        }

        boolean driverCommandingMove = Math.hypot(driverInputs[0], driverInputs[1]) > 0.05;

        if (canShoot && barelyMoving && !driverCommandingMove) {
            if (!isAiming) {
                brakeTimer.restart();
                isAiming = true;
            }
        } else {
            brakeTimer.restart();
            isAiming = false;
        }

        if (brakeTimer.hasElapsed(SwerveConstants.kBrakeTime) && canShoot && barelyMoving && !driverCommandingMove) {
            drivetrain.setControl(brakeRequest);
            return;
        }

        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose, drivetrain.getState().Speeds);

        if (targetLocation == null) {
            drivetrain.setControl(driveRequest
                .withVelocityX(driverInputs[0])
                .withVelocityY(driverInputs[1])
                .withTargetDirection(robotPose.getRotation()));
            return;
        }

        Rotation2d targetHeading = targetLocation.minus(robotPose.getTranslation())
            .getAngle();

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            targetHeading = targetHeading.minus(new Rotation2d(Math.toRadians(180.0)));
        }

        drivetrain.setControl(driveRequest
            .withVelocityX(driverInputs[0])
            .withVelocityY(driverInputs[1])
            .withTargetDirection(targetHeading));
    }
}