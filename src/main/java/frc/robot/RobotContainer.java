// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Notifier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SwerveConstants;
 import frc.robot.commands.AimShooter;
 import frc.robot.commands.AutoAlign;
import frc.robot.commands.LaunchFuelSim;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.FuelSim;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0).withRotationalDeadband(0) // Deadband handled manually for better feel
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

     private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    private final Telemetry logger = new Telemetry(CommandSwerveDrivetrain.MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    // Notifier to publish joystick + requested drivetrain signals for AdvantageScope
    private final Notifier advScopeLogger;

    public final Intake intake = new Intake();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final Supplier<Pose2d> poseSupplier = () -> {
        var state = drivetrain.getState();
        // state.Pose is the robot Pose2d from the drivetrain state
        return state.Pose;
    };

    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier = () -> {
        var state = drivetrain.getState();
        var s = state.Speeds;
        return new ChassisSpeeds(s.vxMetersPerSecond, s.vyMetersPerSecond, s.omegaRadiansPerSecond);
    };
    
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //fuel
        FuelSim.getInstance().spawnStartingFuel();

        FuelSim.getInstance().registerRobot(
        0.900, // from left to right
        0.900, // from front to back
        0.3, // from floor to top of bumpers
        poseSupplier, // Supplier<Pose2d> of robot pose
        fieldSpeedsSupplier); // Supplier<ChassisSpeeds> of field-centric chassis speeds

        FuelSim.getInstance().registerIntake(
        0.2722, 0.8, -0.45, 0.45, // robot-centric coordinates for bounding box
        driver.leftBumper()); // (optional) BooleanSupplier for whether the intake should be active at a given moment

        FuelSim.getInstance().start();

    // Start AdvantageScope logging at 50 Hz so the CTRE sim tools can use
    // joystick inputs and commanded velocities to run a simulation.
    advScopeLogger = new Notifier(() -> logForAdvantageScope());
        advScopeLogger.startPeriodic(0.02);

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
                    double[] drives = CommandSwerveDrivetrain.joyStickPolar(driver, 2);

                    return drive.withVelocityX(drives[0]) // Drive forward with negative Y (forward)
                        .withVelocityY(drives[1]) // Drive left with negative X (left)
                        .withRotationalRate(drives[2]); // Drive counterclockwise with negative X (left)
                })
        );

        intake.setDefaultCommand(intake.intakeDefault());


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

         driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
         driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.x().whileTrue(new AutoAlign(drivetrain));
        driver.y().whileTrue(new AimShooter(drivetrain, faceAngle, driver));

        //intake
        driver.rightTrigger().onTrue(intake.retractIntake());
        driver.leftBumper().whileTrue(intake.runIntake());
        

        //shooter
        driver.rightBumper().whileTrue(new LaunchFuelSim(drivetrain, 0.762));

        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        ); 

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a singl     e log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
/*          driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
 */ 
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    /**
     * Periodically publish joystick axes and commanded drivetrain velocities to
     * the CTRE SignalLogger so AdvantageScope can run a simulation driven by
     * operator inputs.
     */
    private void logForAdvantageScope() {
        // Raw joystick axes
        SignalLogger.writeDouble("Driver/LeftX", driver.getLeftX());
        SignalLogger.writeDouble("Driver/LeftY", driver.getLeftY());
        SignalLogger.writeDouble("Driver/RightX", driver.getHID().getRawAxis(2));
        SmartDashboard.putNumber("RIGHT X", driver.getHID().getRawAxis(2));
        SmartDashboard.putNumber("Right Y",  driver.getHID().getRawAxis(3));

        SmartDashboard.putBoolean("left bump", driver.leftBumper().getAsBoolean());
        SignalLogger.writeDouble("Driver/LeftTrigger", driver.getLeftTriggerAxis());
        SignalLogger.writeDouble("Driver/RightTrigger", driver.getRightTriggerAxis());
        SmartDashboard.putBoolean("aim", driver.y().getAsBoolean());  
        SmartDashboard.putBoolean("align", driver.x().getAsBoolean());  

        // Commanded chassis velocities (same mapping used in default command)
        double[] drives = CommandSwerveDrivetrain.joyStickPolar(driver, 2);
        SignalLogger.writeDouble("Drivetrain/CmdVx", drives[0]);
        SignalLogger.writeDouble("Drivetrain/CmdVy", drives[1]);
        SignalLogger.writeDouble("Drivetrain/CmdOmega", drives[2]);
        
        // Actual drivetrain state for comparison in AdvantageScope
        var state = drivetrain.getState();
        SignalLogger.writeDouble("Drivetrain/PoseX", state.Pose.getX());
        SignalLogger.writeDouble("Drivetrain/PoseY", state.Pose.getY());
        SignalLogger.writeDouble("Drivetrain/HeadingDeg", state.Pose.getRotation().getDegrees());
        SignalLogger.writeDouble("Drivetrain/Vx", state.Speeds.vxMetersPerSecond);
        SignalLogger.writeDouble("Drivetrain/Vy", state.Speeds.vyMetersPerSecond);
        SignalLogger.writeDouble("Drivetrain/Omega", state.Speeds.omegaRadiansPerSecond);
    }
}