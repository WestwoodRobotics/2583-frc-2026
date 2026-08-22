package frc.robot.commands;


import static edu.wpi.first.units.Units.Rotation;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final StructPublisher<Pose3d> turretPub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getStructTopic("Aim/TurretAngle", Pose3d.struct)
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
        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose);
        if (targetLocation == null) {
            distancePub.set(0);
            return;
        }


        Double robotHeading = robotPose.getRotation().getDegrees();
        double distance = shooterPose.getTranslation().getDistance(targetLocation);
       
        distancePub.set(distance);


        SmartDashboard.putBoolean("Distance to Target", m_shooter.isAutoAimEnabled());
        if (!m_shooter.isAutoAimEnabled()) {
            return;
        }


        Double flywheelRPS = MathUtil.clamp(
            4.9665 * distance + 20.5166 + m_shooter.m_delta,
            0.0,
            ShooterConstants.kMaxFlywheelRPS
        );


        Double hoodAngle = 68.0;


        Rotation2d turretAngleFieldRel = Rotation2d.fromDegrees((Math.atan2(targetLocation.getY() - shooterPose.getY(), targetLocation.getX() - shooterPose.getX())/Math.PI*180.0)+180.0);
        Rotation2d turretRobotRel = turretAngleFieldRel.minus(Rotation2d.fromDegrees(robotHeading));
        Double turretPos = ((turretRobotRel.getDegrees() % 360 + 360) % 360)/360;
        SmartDashboard.putNumber("Turret Angle", turretPos);
        Pose3d turret = new Pose3d(shooterPose.getX(), shooterPose.getY(), 1.5, new Rotation3d(0, 0, turretAngleFieldRel.getRadians()));
        turretPub.set(turret);
        if (DriverStation.isTeleop()) {
            if (m_shooter.getHoodState()) {
                if (!GetTargetLocation.inZone()) {
                    hoodAngle = ShooterConstants.kTrueMinAngle;
                }
            } else {
                hoodAngle = ShooterConstants.kMaxAngle;
                if (GetTargetLocation.inZone()) {
                    if (m_shooter.getDormantMode()) {
                        flywheelRPS = ShooterConstants.kZoneDormantVel;
                    }
                }
                else {
                    flywheelRPS = ShooterConstants.kPassingDormantVel;
                }
            }
        }


        if (flywheelRPS == null || hoodAngle == null || turretPos == null) {
            // If distance is out of bounds of our mapping, do not adjust shooter
            m_shooter.setFlywheelVelocity(0.0);
            m_shooter.setHoodAngle(ShooterConstants.kMaxAngle);
            m_shooter.setTurretPosition(0.0);
            return;
        }


        m_shooter.setHoodAngle(hoodAngle);
        m_shooter.setFlywheelVelocity(flywheelRPS);
        m_shooter.setTurretPosition(turretPos);
    }
   
}

