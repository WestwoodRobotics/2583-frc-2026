package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to automatically align the robot to the left tower using CTRE's LinearPath.
 */
public class AutoAlign extends Command {
    private CommandSwerveDrivetrain m_drivetrain;

    private LinearPath path;
    private LinearPath.State current;
    private Pose2d target;

    private Timer timer;
    private double currentTime;
    private double deltaTime;

    public AutoAlign(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        timer = new Timer();

        target = SwerveConstants.leftTower;
        
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // Generate a path from the current robot pose to the target leftTower pose
        path = new LinearPath(
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxVel, SwerveConstants.alignMaxAccel),
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxOmega, SwerveConstants.alignMaxAlpha)
        );
        current = new LinearPath.State(
            m_drivetrain.getState().Pose,
            m_drivetrain.getState().Speeds
        );
        timer.restart();
        currentTime = 0.0;
    }

    @Override
    public void execute() {
        deltaTime = timer.get() - currentTime;
        currentTime = timer.get();

        current = path.calculate(deltaTime, current, target);
        m_drivetrain.setControl(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(current.speeds)
        );
    }

    @Override
    public boolean isFinished() {
        return path.isFinished(currentTime);
    }
}