package frc.robot.commands;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GetTargetLocation;
import frc.robot.utils.ShotParam;

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

        if (DriverStation.isAutonomousEnabled() || !m_shooter.isAutoAimEnabled()) {
            return;
        }
        ShotParam shotParam = getShotParam(distance);
        m_shooter.setHoodAngle(shotParam.angle);
        m_shooter.setFlywheelVelocity(shotParam.velocity);
    }

    public ShotParam getShotParam(double distance) {
        TreeMap<Double, ShotParam> map = ShooterConstants.kDistanceToShotParam;
        Map.Entry<Double, ShotParam> floor = map.floorEntry(distance);
        Map.Entry<Double, ShotParam> ceil = map.ceilingEntry(distance);

        if (floor == null && ceil == null) return new ShotParam(ShooterConstants.kMinAngle, 0);
        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();

        ShotParam floorVal = floor.getValue();
        ShotParam ceilVal = ceil.getValue();

        if (floor.getKey().equals(ceil.getKey())) return floorVal;

        double t = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
        double angle = floorVal.angle + t * (ceilVal.angle - floorVal.angle);
        double velocity = floorVal.velocity + t * (ceilVal.velocity - floorVal.velocity);

        angle = MathUtil.clamp(angle, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
        return new ShotParam(angle, velocity);
    }
}
