package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to automatically align the robot to the left tower using CTRE's LinearPath.
 */
public class AlignCornerShot extends Command {
    private CommandSwerveDrivetrain m_drivetrain;

    private LinearPath path;
    private LinearPath.State current;
    private Pose2d target;
    private final SwerveRequest.ApplyFieldSpeeds m_fieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private Timer timer;
    private double currentTime;
    private double deltaTime;

    public AlignCornerShot(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        timer = new Timer();

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            path = null;
            return;
        }
        boolean isBlue = allianceOpt.orElse(Alliance.Blue) == Alliance.Blue;
        Pose2d robotPose = m_drivetrain.getState().Pose;

        boolean inZone = isBlue ? 
            robotPose.getX() < SwerveConstants.allianceZoneWidth : 
            robotPose.getX() > (SwerveConstants.fieldWidth - SwerveConstants.allianceZoneWidth);

        if (!inZone) {
            path = null;
            return;
        }

        boolean isTop = robotPose.getY() > (SwerveConstants.fieldLength / 2.0);
        
        if (isBlue) {
            if (isTop) {
                target = new Pose2d(SwerveConstants.blueCornerX, SwerveConstants.upperCornerY, new Rotation2d(SwerveConstants.upperBlueCornerAngle));
            } else {
                target = new Pose2d(SwerveConstants.blueCornerX, SwerveConstants.lowerCornerY, new Rotation2d(SwerveConstants.lowerBlueCornerAngle));
            }
        } else {
            if (isTop) {
                target = new Pose2d(SwerveConstants.redCornerX, SwerveConstants.upperCornerY, new Rotation2d(SwerveConstants.upperRedCornerAngle));
            } else {
                target = new Pose2d(SwerveConstants.redCornerX, SwerveConstants.lowerCornerY, new Rotation2d(SwerveConstants.lowerRedCornerAngle));
            }
        }

        path = new LinearPath(
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxVel, SwerveConstants.alignMaxAccel),
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxOmega, SwerveConstants.alignMaxAlpha)
        );
        current = new LinearPath.State(
            robotPose,
            m_drivetrain.getState().Speeds
        );
        // Initialize path duration by calling calculate once
        current = path.calculate(0, current, target);
        timer.restart();
        currentTime = 0.0;
    }

    @Override
    public void execute() {
        if (path == null) return;

        double newTime = timer.get();
        deltaTime = newTime - currentTime;
        currentTime = newTime;

        current = path.calculate(deltaTime, current, target);
        m_drivetrain.setControl(
            m_fieldSpeeds.withSpeeds(current.speeds)
        );
    }

    @Override
    public boolean isFinished() {
        if (path == null) return true;
        return false;
    }
}