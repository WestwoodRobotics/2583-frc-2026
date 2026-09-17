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
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GetTargetLocation;


public class AdjustShooter extends Command {


    private Shooter m_shooter;
    private CommandSwerveDrivetrain m_drivetrain;
    private CommandXboxController m_driver;


    private final DoublePublisher distancePub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Aim/DistanceToTarget")
        .publish();
    private final DoublePublisher m_turretDesiredAngle = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Turret/DesiredAngle")
        .publish();






    public AdjustShooter(Shooter shooter, CommandSwerveDrivetrain drivetrain, CommandXboxController driver) {
        m_shooter = shooter;
        m_drivetrain = drivetrain;
        m_driver = driver;
        addRequirements(m_shooter);
    }


    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Pose2d shooterPose = robotPose.plus(SwerveConstants.robotToShooter);


        Double robotAngle = robotPose.getRotation().getDegrees();


        boolean holdingShoot = m_driver.rightTrigger().getAsBoolean();


        m_shooter.setHood(holdingShoot);
   


        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose);
        if (targetLocation == null) {
            distancePub.set(0);
            return;
        }


        double distance = shooterPose.getTranslation().getDistance(targetLocation);
       
        distancePub.set(distance);


        if (!m_shooter.isAutoAimEnabled()) {
            return;
        }


        Double flywheelRPS = MathUtil.clamp(
            4.9665 * distance + 20.5166 + m_shooter.m_delta,
            0.0,
            ShooterConstants.kMaxFlywheelRPS
        );
        Double hoodAngle = 68.0;


        Double shooterToTargetAngle = targetLocation.minus(shooterPose.getTranslation()).getAngle().getDegrees();
        Double TurretAngle = MathUtil.inputModulus(shooterToTargetAngle - robotAngle , -180,180);
        Double desiredTurretAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red? TurretAngle : -TurretAngle;


        if (DriverStation.isTeleop()) {
            if (m_shooter.getHoodState()) {
                if (holdingShoot && !GetTargetLocation.inZone()) {
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


        if (flywheelRPS == null || hoodAngle == null) {
            // If distance is out of bounds of our mapping, do not adjust shooter
            m_shooter.setFlywheelVelocity(0.0);
            m_shooter.setHoodAngle(ShooterConstants.kMaxAngle);
            return;
        }


        m_shooter.setHoodAngle(hoodAngle);
        m_shooter.setFlywheelVelocity(flywheelRPS);
        m_shooter.setTurretAngle(desiredTurretAngle);
       
    }
   
}

