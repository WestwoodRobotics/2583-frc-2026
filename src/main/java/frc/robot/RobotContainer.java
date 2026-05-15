// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.commands.AimSwerve;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.IntakeShoot;
import frc.robot.commands.LockHeading;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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
    public final LED led = new LED(drivetrain, driver);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Autonomous configurations
        configureAutonomousCommands();
        autoChooser = AutoBuilder.buildAutoChooser("PreloadAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        faceAngle.HeadingController.setPID(SwerveConstants.aimKp, SwerveConstants.aimKi, SwerveConstants.aimKd);
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand().ignoringDisable(true));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                    CommandSwerveDrivetrain.joyStickPolar(driverInputs, driver);

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
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(
            () -> shooter.setAutoAim(true))
        );
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(
            () -> shooter.setAutoAim(false))
        );

        driver.a().whileTrue(new AimSwerve(drivetrain, shooter, faceAngle, brake, driver));

        driver.x().whileTrue(new LockHeading(drivetrain, faceAngle, driver, 45.0).alongWith(intake.partialRetract()))
            .onFalse(Commands.runOnce(() -> intake.setPivotPosition(IntakeConstants.kPivotOut)));
        
        driver.y().whileTrue(Commands.run(() -> {
            shooter.setFlywheelVelocity(ShooterConstants.kYFlywheelVel);
            shooter.setHoodAngle(ShooterConstants.kYHoodAngle);
        }, shooter));
        
        driver.b().whileTrue(new LockHeading(drivetrain, faceAngle, driver, 0.0));

        driver.rightTrigger().whileTrue(Commands.parallel(
            transfer.shootCommand(true),
            new AimSwerve(drivetrain, shooter, faceAngle, brake, driver),
            new IntakeShoot(intake, driver)
        ));

        // Run intake while holding left trigger
        driver.leftTrigger().and(driver.rightTrigger().negate()).whileTrue(intake.runIntake());

        driver.leftBumper().whileTrue(Commands.runEnd(
            () -> {
                intake.setPivotPosition(IntakeConstants.kPivotDepot);
                intake.setRollerVelocity(IntakeConstants.kRollerIntakingVel);
            },
            () -> intake.setPivotPosition(IntakeConstants.kPivotOut),
            intake
        ));

        driver.rightBumper().whileTrue(
            transfer.shootCommand(false)
            .alongWith(new IntakeShoot(intake, driver))
        );

        driver.start().whileTrue(
            drivetrain.applyRequest(() -> {
                CommandSwerveDrivetrain.joyStickPolar(driverInputs, driver);

                return drive.withVelocityX(driverInputs[0] * SwerveConstants.kSlowmodeK)
                    .withVelocityY(driverInputs[1] * SwerveConstants.kSlowmodeK)
                    .withRotationalRate(driverInputs[2]);
            })
        );

        operator.x().onTrue(Commands.runOnce(shooter::toggleAutoAim, shooter));
        operator.y().onTrue(intake.fullRetract());
        operator.b().onTrue(transfer.shootTimeCommand());
        operator.a().whileTrue(Commands.runEnd(
            () -> intake.setPivotPosition(IntakeConstants.kPivotUp),
            () -> intake.setPivotPosition(IntakeConstants.kPivotOut),
            intake
        ));

        operator.rightTrigger().onTrue(Commands.runOnce(
            () -> shooter.changeHoodAngle(-ShooterConstants.kManualHoodInc),
            shooter));
        operator.leftTrigger().onTrue(Commands.runOnce(
            () -> shooter.changeHoodAngle(ShooterConstants.kManualHoodInc),
            shooter));
        operator.rightBumper().onTrue(Commands.runOnce(
            () -> shooter.changeFlywheelVelocity(ShooterConstants.kManualFlywheelInc), shooter));
        operator.leftBumper().onTrue(Commands.runOnce(
            () -> shooter.changeFlywheelVelocity(-ShooterConstants.kManualFlywheelInc), shooter));

        operator.povLeft().onTrue(Commands.runOnce(shooter::resetHoodPosition, shooter).ignoringDisable(true));
        operator.povRight().onTrue(Commands.runOnce(intake::resetPivot, intake).ignoringDisable(true));
        operator.povDown().onTrue(Commands.runOnce(() -> shooter.changeDelta(-0.5)));
        operator.povUp().onTrue(Commands.runOnce(() -> shooter.changeDelta(0.5)));

        operator.rightStick().onTrue(Commands.runOnce(() -> {
            Pose2d pose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? new Pose2d(SwerveConstants.kResetX, SwerveConstants.kResetY, new Rotation2d(0.0))
                : new Pose2d(SwerveConstants.fieldWidth - SwerveConstants.kResetX, SwerveConstants.kResetY, new Rotation2d(Math.PI));
            drivetrain.resetPose(pose);
        }).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.x().whileTrue(intake.sysIdDynamic(Direction.kForward));
        // driver.y().whileTrue(intake.sysIdDynamic(Direction.kReverse));
        // driver.b().whileTrue(intake.sysIdQuasistatic(Direction.kForward));
        // driver.a().whileTrue(intake.sysIdQuasistatic(Direction.kReverse));

        // driver.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // driver.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureAutonomousCommands() {

        NamedCommands.registerCommand("FlywheelOn", Commands.runOnce(() -> shooter.setAutoAim(true)));
        NamedCommands.registerCommand("FlywheelOff", Commands.runOnce(() -> shooter.setAutoAim(false)));
        NamedCommands.registerCommand("StopTransfer", transfer.stopTransfer());
    
        NamedCommands.registerCommand("RunIntake", intake.runIntake());
        NamedCommands.registerCommand("IntakeDepot", Commands.runEnd(
            () -> {
                intake.setPivotPosition(IntakeConstants.kPivotDepot);
                intake.setRollerVelocity(IntakeConstants.kRollerIntakingVel);
            },
            () -> intake.setPivotPosition(IntakeConstants.kPivotOut),
            intake
        ));
        NamedCommands.registerCommand("PartialRetract", intake.partialRetract());

        NamedCommands.registerCommand("Shoot", Commands.race(
            new WaitCommand(3.0),
            transfer.shootCommand(true),
            new AimSwerve(drivetrain, shooter, faceAngle, brake, driver),
            new IntakeShoot(intake, driver)
        ));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}