package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import javax.sound.sampled.Line;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FuelSim;

public class LaunchFuelSim extends Command{
    private double velocity;
    private double angle;
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d robotpose;
    private double launchheight;
    private double disttohub;

    private final Timer shootTimer = new Timer();
    private static final double SHOOT_INTERVAL = 0.0833; // Time between shots in seconds (0.05 = 20 shots/sec)

    public LaunchFuelSim(CommandSwerveDrivetrain drivetrain , double launchheight) {
        this.drivetrain = drivetrain;
        this.launchheight = launchheight;
    }
    @Override
    public void initialize() {
        shootTimer.restart();

    }

    @Override
    public void execute() {

        robotpose = drivetrain.getState().Pose;
        

        
        
        double dx = (robotpose.getX() - SwerveConstants.blueHub.getX()) * 39.37 ;
        double dy = (robotpose.getY() - SwerveConstants.blueHub.getY()) * 39.37 ;

        disttohub = Math.hypot(dx, dy);
        angle = Math.min(Math.max(70-(27.0/250.0)*(disttohub-40), 47),70);

        double angleRad = Math.toRadians(angle);

        double denominator = (2 * Math.pow(Math.cos(angleRad),2)*(disttohub*Math.tan(angleRad)-(72-30)));

        velocity = Math.sqrt((386.09*Math.pow(disttohub, 2))/denominator) / 39.37; //m/s


        

        if (shootTimer.hasElapsed(SHOOT_INTERVAL)) {
            shootTimer.reset();

            if(FuelSim.getInstance().hasFuel()){ 
                Translation3d initialpos = new Translation3d(
                    robotpose.getX(),
                    robotpose.getY(),
                    launchheight
                );

                double heading = robotpose.getRotation().getRadians();
                double horizSpeed = velocity * Math.cos(angleRad);
                double vx = horizSpeed * Math.cos(heading);
                double vy = horizSpeed * Math.sin(heading);
                double vz = velocity * Math.sin(angleRad);
                Translation3d velovector = new Translation3d(vx, vy, vz);
                FuelSim.getInstance().consumeFuel();
                FuelSim.getInstance().spawnFuel(initialpos, velovector);
            }
        } 
       
        


        /* SmartDashboard.putNumber("Launch Velocity", velocity);
        SmartDashboard.putNumber("Launch Angle", angle);
        SmartDashboard.putNumber("pose inches x", dx);
        SmartDashboard.putNumber("pose inches y", dy); */
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shootTimer.stop();
    }
    
}
