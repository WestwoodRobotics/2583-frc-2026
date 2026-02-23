package frc.robot.commands;

import java.util.Map;
import java.util.TreeMap;

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
        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose);

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
        TreeMap<Double, Double> map = ShooterConstants.kDistanceToRPS;
        Map.Entry<Double, Double> floor = map.floorEntry(distance);
        Map.Entry<Double, Double> ceil = map.ceilingEntry(distance);

        if (floor == null && ceil == null) return 0.0;
        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();

        Double floorVal = floor.getValue();
        Double ceilVal = ceil.getValue();

        if (floor.getKey().equals(ceil.getKey())) return floorVal;

        double t = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
        double rps = Math.min(floorVal + t * (ceilVal - floorVal), ShooterConstants.kMaxFlywheelRPS);

        return rps;
    }

    public double getHoodAngle(double distance) {
        Double angle = ShooterConstants.kMaxAngle - (ShooterConstants.kMaxAngle - ShooterConstants.kMinAngle) / (ShooterConstants.kFarDistance - ShooterConstants.kNearDistance) * (distance - 40);
        return MathUtil.clamp(angle, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
    }
}
