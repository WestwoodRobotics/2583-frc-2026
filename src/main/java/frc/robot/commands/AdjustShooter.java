package frc.robot.commands;


import static edu.wpi.first.units.Units.Rotation;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
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

    private final BooleanPublisher wrapPub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("Aim/TurretWrap")
        .publish();
  
    private final DoubleSubscriber m_turretActualAngle = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Turret/ActualAngle")
        .subscribe(0.0);
    
    private final DoubleSubscriber m_turretDesiredAngle = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Turret/DesiredAngle")
        .subscribe(0.0);

    private final StructPublisher<Pose2d> turret = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getStructTopic("Turret/Turretpos", Pose2d.struct)
        .publish();

    private static final StructPublisher<Pose2d> target = NetworkTableInstance.getDefault()
        .getTable("TargetLoc")
        .getStructTopic("TargetLoc/target", Pose2d.struct)
        .publish();

    private final DoubleSubscriber m_flywheelInterceptDelta = NetworkTableInstance.getDefault().getDoubleTopic("Flywheel/InterceptDelta").subscribe(0.0);


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
        // SmartDashboard.putBoolean("driverx", m_driver.x().getAsBoolean());
        // SmartDashboard.putBoolean("holdingshoot",holdingShoot);

        m_shooter.setHood(holdingShoot);
   
        Translation2d targetLocation = !Shooter.SOTMToggle? 
        GetTargetLocation.getTargetLocation(robotPose) :
        GetTargetLocation.getTargetLocation(robotPose, m_drivetrain.getState().Speeds);
        
      
        if (targetLocation == null) {
            distancePub.set(0);
            return;
        }
        target.set(new Pose2d(targetLocation, Rotation2d.fromDegrees(0)));


        double distance = shooterPose.getTranslation().getDistance(targetLocation);
       
        distancePub.set(distance);
        Double flywheelRPS = ShooterConstants.kDistanceToRPS.get(distance);
        Double hoodAngle = ShooterConstants.kDistanceToAngle.get(distance);


        Double shooterToTargetAngle = targetLocation.minus(shooterPose.getTranslation()).getAngle().getDegrees();
        double turretAngle = shooterToTargetAngle - robotAngle + ShooterConstants.kTurretOffset;
       /*  double turretAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? -rawTurretAngle
            : rawTurretAngle; */

       
        double base = MathUtil.inputModulus(turretAngle, -180.0, 180.0);
        // SmartDashboard.putNumber("base", base);
        double acceptLimit = ShooterConstants.kTurretMaxAngle - ShooterConstants.kTurretWrapDeadbandDegrees;
        double current = m_turretActualAngle.get(); //USE DESIRED ONLY FOR SIM
        double currentDes = m_turretDesiredAngle.get();

        // SmartDashboard.putNumber("error", Math.abs(Math.abs(currentDes)-Math.abs(current)));
        wrapPub.set(!(Math.abs(currentDes-current)
        < ShooterConstants.kTurretWrapDeadbandDegrees));
        
        // SmartDashboard.putNumber("current", current);
        double desiredAngle = base;
        double bestDist = Math.abs(base - current);
        // SmartDashboard.putNumber("bestDist", bestDist);
        double basePositive = base+360.0;
        double baseNegative = base-360.0;
        
        if(Math.abs(basePositive) < acceptLimit){
            double dist = Math.abs(basePositive - current);
            if(dist < bestDist){
                bestDist = dist;
                desiredAngle = basePositive;
                // SmartDashboard.putBoolean("basepositive", true);
            }
        }

        if(Math.abs(baseNegative) < acceptLimit){
            double dist2 = Math.abs(baseNegative - current);
            if(dist2 < bestDist){
                bestDist = dist2;
                desiredAngle = baseNegative;
                // SmartDashboard.putBoolean("basenegative", true);

            }
        }
       
        

        Double desiredTurretAngle = desiredAngle;
        if(DriverStation.isTeleop()){
            m_shooter.setTurretAngle(desiredTurretAngle);
        } if(DriverStation.isAutonomous()) {
            m_shooter.setTurretAngle(desiredTurretAngle-20);
        }
        turret.set(new Pose2d(robotPose.getX(), robotPose.getY(), Rotation2d.fromDegrees(robotPose.getRotation().getDegrees() + Shooter.m_desiredTurretAngle)));

        if (!m_shooter.isAutoAimEnabled()) {
            return;
        }




        if (DriverStation.isTeleop()) {
            if (m_shooter.getHoodState()) {
                if (!GetTargetLocation.inZone()) {
                    hoodAngle = ShooterConstants.kTrueMinAngle;
                   
                }
            } else {
                hoodAngle = ShooterConstants.kMaxAngle;
                // if (GetTargetLocation.inZone()) {
                //     if (m_shooter.getDormantMode()) {
                //         flywheelRPS = ShooterConstants.kZoneDormantVel;
                //     }
                // }
                // else {
                //     flywheelRPS = ShooterConstants.kPassingDormantVel;
                // }
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
       
    }
   
}

