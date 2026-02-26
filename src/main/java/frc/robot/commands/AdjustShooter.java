package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GetTargetLocation;

public class AdjustShooter extends Command {

    private Shooter m_shooter;
    private CommandSwerveDrivetrain m_drivetrain;


    public AdjustShooter(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Pose2d shooterPose = robotPose.transformBy(SwerveConstants.shooterToRobot);
        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose, m_drivetrain.getState().Speeds);

        if (targetLocation == null) {
            m_shooter.setFlywheelVelocity(0.0);
            return;
        }

        double distance = shooterPose.getTranslation().getDistance(targetLocation);
        double rps = getFlywheelRPS(distance);
        double angle = getHoodAngle(distance);
        m_shooter.setHoodAngle(angle);
        m_shooter.setFlywheelVelocity(rps);
    }

    public double getFlywheelRPS(double distance) {
        return ShooterConstants.kDistanceToRPS.get(distance);
    }

    public double getHoodAngle(double distance) {
        Double angle = ShooterConstants.kMaxAngle - (ShooterConstants.kMaxAngle - ShooterConstants.kMinAngle) / (ShooterConstants.kFarDistance - ShooterConstants.kNearDistance) * (distance - ShooterConstants.kNearDistance);
        return MathUtil.clamp(angle, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
    }
}
