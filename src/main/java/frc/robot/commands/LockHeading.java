package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LockHeading extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle faceAngle;
    private final CommandXboxController controller;
    private final double lockHeadingOffset;
    private final double[] driverInputs = new double[3];

    public LockHeading(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle faceAngle, CommandXboxController controller, double lockHeadingOffset) {
        this.drivetrain = drivetrain;
        this.faceAngle = faceAngle;
        this.controller = controller;
        this.lockHeadingOffset = lockHeadingOffset;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        CommandSwerveDrivetrain.joyStickPolar(driverInputs, controller);
        
        double currentDeg = drivetrain.getState().Pose.getRotation().getDegrees();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            currentDeg += 180.0;
        }
        double closestDeg = Math.round((currentDeg - lockHeadingOffset) / 90.0) * 90.0 + lockHeadingOffset;

        drivetrain.setControl(faceAngle
            .withVelocityX(driverInputs[0])
            .withVelocityY(driverInputs[1])
            .withTargetDirection(Rotation2d.fromDegrees(closestDeg)));
    }
}