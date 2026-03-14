// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimSwerve;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.AlignCornerShot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utils.GetHubStatus;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0).withRotationalDeadband(0) // Deadband handled manually for better feel
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(0).withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private double[] driverInputs = new double[3];

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();  
      
    private final Telemetry logger = new Telemetry(CommandSwerveDrivetrain.MaxSpeed, drivetrain.getPigeon2());

    public final Intake intake = new Intake();
    public final Vision vision = new Vision(drivetrain);
    public final Transfer transfer = new Transfer();
    public final Shooter shooter = new Shooter();
    public final LED led = new LED(drivetrain, driver, operator);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureEventTrigger();

        configureBindings();

        faceAngle.HeadingController.setPID(SwerveConstants.aimKp, SwerveConstants.aimKi, SwerveConstants.aimKd);
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                    CommandSwerveDrivetrain.joyStickPolar(driverInputs, driver, 2);

                    return drive.withVelocityX(driverInputs[0]) // Drive forward with negative Y (forward)
                        .withVelocityY(driverInputs[1]) // Drive left with negative X (left)
                        .withRotationalRate(driverInputs[2]); // Drive counterclockwise with negative X (left)
                })
        );

        intake.setDefaultCommand(intake.intakeDefault());
        transfer.setDefaultCommand(transfer.defaultCommand());
        shooter.setDefaultCommand(new AdjustShooter(shooter, drivetrain));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(new AimSwerve(drivetrain, faceAngle, driver));

        driver.x().whileTrue(drivetrain.applyRequest(() -> {
            CommandSwerveDrivetrain.joyStickPolar(driverInputs, driver, 2);

            Rotation2d currentRot = drivetrain.getState().Pose.getRotation();

            if (DriverStation.getAlliance().get() == Alliance.Red) {
                currentRot = currentRot.plus(new Rotation2d(Math.PI));
            }
            double currentDeg = currentRot.getDegrees();
            double closestDiagonalDeg = Math.round((currentDeg - 45) / 90.0) * 90.0 + 45;

            return faceAngle
                .withVelocityX(driverInputs[0])
                .withVelocityY(driverInputs[1])
                .withTargetDirection(Rotation2d.fromDegrees(closestDiagonalDeg));

        }).alongWith(intake.partialRetract()))
            .onFalse(Commands.runOnce(() -> intake.setPivotPosition(IntakeConstants.pivotOut)));
        
        driver.y().whileTrue(new AlignCornerShot(drivetrain));
        driver.b().whileTrue(intake.shootCommand());

        driver.rightBumper().onTrue(Commands.runOnce(shooter::toggleAutoAim, shooter)
            .andThen(Commands.runOnce(() -> {
                shooter.setFlywheelVelocity(40.83051602354075);
                shooter.setHoodAngle(57.0);
        })));

        driver.rightTrigger().whileTrue(transfer.shootCommand());

        // Run intake while holding left trigger
        driver.leftTrigger().whileTrue(intake.runIntake(false));

        driver.leftBumper().whileTrue(intake.runIntake(true));

        operator.x().onTrue(Commands.runOnce(shooter::toggleAutoAim, shooter)
            .andThen(Commands.runOnce(() -> shooter.setFlywheelVelocity(0.0), shooter)));
        operator.y().onTrue(intake.fullRetract());
        operator.b().onTrue(intake.partialRetract());
        operator.a().whileTrue(transfer.reverseCommand());
        
        operator.rightTrigger().whileTrue(Commands.startEnd(
            () -> shooter.setHoodVoltage(ShooterConstants.kManualHoodVolts),
            () -> shooter.setHoodVoltage(0),
            shooter));
        operator.leftTrigger().whileTrue(Commands.startEnd(
            () -> shooter.setHoodVoltage(-ShooterConstants.kManualHoodVolts),
            () -> shooter.setHoodVoltage(0),
            shooter));

        operator.rightBumper().onTrue(Commands.runOnce(
            () -> shooter.changeFlywheelVelocity(ShooterConstants.kManualFlywheelInc), shooter));
        operator.leftBumper().onTrue(Commands.runOnce(
            () -> shooter.changeFlywheelVelocity(-ShooterConstants.kManualFlywheelInc), shooter));

        operator.povDown().onTrue(Commands.runOnce(shooter::resetHoodPosition, shooter).ignoringDisable(true));
        operator.povRight().onTrue(Commands.runOnce(intake::resetPivot, intake).ignoringDisable(true));
        operator.povLeft().onTrue(Commands.runOnce(GetHubStatus::togglePracticeMode));

        operator.rightStick().onTrue(Commands.runOnce(() -> {
            Pose2d pose = DriverStation.getAlliance().get() == Alliance.Blue ? new Pose2d(3.560, 4.029, new Rotation2d(0.0)) : new Pose2d(SwerveConstants.fieldWidth - 3.560, 4.029, new Rotation2d(Math.PI));
            drivetrain.resetPose(pose);
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.x().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // driver.y().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        // driver.b().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // driver.a().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

        // driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureEventTrigger() {

        new EventTrigger("Shoot").whileTrue(transfer.shootCommand().alongWith(intake.shootCommand()));

        new EventTrigger("RunIntake").whileTrue(intake.runIntake(false));
        new EventTrigger("PartialRetract").onTrue((intake.partialRetract()));

//        new EventTrigger("AimSwerve").onTrue(Commands.deadline(new AimSwerve(drivetrain, faceAngle, driver), new WaitCommand(2.0)));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}