package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GetTargetLocation;

public class AdjustShooter extends Command {

    private Shooter m_shooter;
    private CommandSwerveDrivetrain m_drivetrain;

    private final DoublePublisher distancePub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Aim/DistanceToTarget")
        .publish();

    public AdjustShooter(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Pose2d shooterPose = robotPose.plus(SwerveConstants.robotToShooter);

        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose, m_drivetrain.getState().Speeds);
        if (targetLocation == null) {
            distancePub.set(0);
            return;
        }

        double distance = shooterPose.getTranslation().getDistance(targetLocation);
       
        distancePub.set(distance);

        if (!m_shooter.isAutoAimEnabled()) {
            return;
        }

        Double flywheelRPS = ShooterConstants.kDistanceToRPS.get(distance);
        Double hoodAngle = ShooterConstants.kDistanceToAngle.get(distance);

        if (flywheelRPS == null || hoodAngle == null) {
            // If distance is out of bounds of our mapping, do not adjust shooter
            m_shooter.setFlywheelVelocity(0.0);
            m_shooter.setHoodAngle(ShooterConstants.kMaxAngle);
            return;
        }

        flywheelRPS = MathUtil.clamp(flywheelRPS, 0, ShooterConstants.kMaxFlywheelRPS);
        m_shooter.setFlywheelVelocity(flywheelRPS);   
        hoodAngle = MathUtil.clamp(hoodAngle, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
        m_shooter.setHoodAngle(hoodAngle);
    }
    
}