package frc.robot.commands;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    public static double headingCorrection;
    public static double perpendicularVel;

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
        double xVel = (m_drivetrain.getState().Speeds.vxMetersPerSecond);
        double yVel = m_drivetrain.getState().Speeds.vyMetersPerSecond;

        double robotHeading = m_drivetrain.getState().RawHeading.getRadians();

        double fieldXVel = xVel * Math.cos(robotHeading) - yVel * Math.sin(robotHeading);
        double fieldYVel = xVel * Math.sin(robotHeading) + yVel * Math.cos(robotHeading);

        Translation2d targetLocation = GetTargetLocation.getTargetLocation(robotPose, m_drivetrain.getState().Speeds);
        if (targetLocation == null) {
            distancePub.set(0);
            return;
        }

        double distance = shooterPose.getTranslation().getDistance(targetLocation);
        // double flywheelRPS = (5.193323 * distance) + 32.47639;
        double flywheelRPS = ShooterConstants.kDistanceToRPS.get(distance);
        double flywheelMS = flywheelRPS * (0.0508) * (2* Math.PI);

        //compensations for the robot movement
        double xCompensation = fieldXVel * Math.cos(-robotHeading); 
        double yCompensation = fieldYVel * Math.sin(-robotHeading);

        //decomposition of ball vector
        double horziontal_Ball_component = flywheelMS * Math.cos(Math.toRadians(60));
        double vertical_Ball_component = flywheelMS * Math.sin(Math.toRadians(60));

        double totalCompensation = xCompensation - yCompensation;

        double Compensated_Horizontal_Ball = horziontal_Ball_component - (totalCompensation * 2.0); //tuning compensation multiplier
        
        SmartDashboard.putNumber("total Compensation", totalCompensation);
        //vertical component is always the same, only thing we have to change is horizontal
        double Compensated_FlywheelMS = Math.sqrt((Math.pow(Compensated_Horizontal_Ball, 2)) + (Math.pow(vertical_Ball_component , 2)));
        double Compensated_FlywheelRPS = (Compensated_FlywheelMS / (0.0508) / (2*Math.PI)) ;

        //angle compensation
        double goalDX = shooterPose.getX() - targetLocation.getX();
        double goalDY =  shooterPose.getY() - targetLocation.getY();

        double unitGoalDX = goalDX / distance;
        double unitGoalDY = goalDY / distance;

        double perpendicularX = -unitGoalDY;
        double perpendicularY = unitGoalDX;

        perpendicularVel = (perpendicularX * fieldXVel) + (perpendicularY * fieldYVel);

        headingCorrection = Math.atan2(perpendicularVel, distance);
       
        distancePub.set(distance);

        if (!m_shooter.isAutoAimEnabled()) {
            return;
        }

        Compensated_FlywheelRPS = MathUtil.clamp(Compensated_FlywheelRPS, 0, ShooterConstants.kMaxFlywheelRPS);
        m_shooter.setFlywheelVelocity(Compensated_FlywheelRPS);   
    }
    
}