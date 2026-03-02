package frc.robot.commands;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;


import org.photonvision.PhotonCamera;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;


public class DriveToObject extends Command{
    private CommandSwerveDrivetrain m_drivetrain;


    private LinearPath path;
    private LinearPath.State current;
    private Collection<Pose2d> targets;


    private List<Pose2d> nears;


    private Pose2d target;


    private PhotonCamera camera;

    private Timer timer;
    private double currentTime;
    private double deltaTime;

    private StructArraySubscriber<Translation3d> fuelSubscriber = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Fuel Simulation/Fuels", Translation3d.struct)
        .subscribe(new Translation3d[0]); 

    public DriveToObject(CommandSwerveDrivetrain drivetrain){
        m_drivetrain = drivetrain;
        timer = new Timer();
    }


    @Override
    public void initialize(){

        targets = new ArrayList<Pose2d>();
        nears = new ArrayList<Pose2d>();
        List<Translation3d> transform = Arrays.asList(fuelSubscriber.get());
        List<Transform3d> transforms = transform.stream().map(t -> new Transform3d(t, new Rotation3d())).toList();

        for(int i = 0; i < transforms.size(); i++){
            Transform3d object3d = transforms.get(i);


            Pose2d object2d = new Pose2d(
                object3d.getX(),
                object3d.getY(),
                object3d.getRotation().toRotation2d()
            );


            targets.add(object2d);
        }

        if(targets.isEmpty()) return;
       
        Iterator<Pose2d> iterator = targets.iterator();

        Pose2d nearest = m_drivetrain.getState().Pose.nearest(new ArrayList<>(targets));


        for(int j = 0; j < targets.size(); j++){


            Pose2d object = iterator.next();
            Translation2d objTranslation = object.getTranslation();

            List<Pose2d> noObj = new ArrayList<>(targets);
            noObj.remove(object);

            if(noObj.isEmpty()) continue;

            if(object.getTranslation().getDistance(nearest.getTranslation()) < 2){
                nears.add(object);
            }
        }


        double totalX = 0;
        double totalY = 0;


        for(int m = 0; m < nears.size(); m++){
            totalX += nears.get(m).getX();
            totalY += nears.get(m).getY();
        }

        if(nears.isEmpty()) return;

        target = new Pose2d(
            totalX/nears.size(),
            totalY/nears.size(),
            new Rotation2d()
        );

        SmartDashboard.putNumberArray("targetpose", new double[]{target.getX(), target.getY(), target.getRotation().getRadians()});
        path = new LinearPath(
            new TrapezoidProfile.Constraints(5.0, 2.0),
            new TrapezoidProfile.Constraints(2.0, 2.0)
        );

        current = new LinearPath.State(
            m_drivetrain.getState().Pose,
            m_drivetrain.getState().Speeds
        );

        timer.restart();
        currentTime = 0.0;
    }




    @Override
    public void execute(){

        SmartDashboard.putBoolean("running is true", true);
        Double newTime = timer.get();
        deltaTime = newTime - currentTime;
        currentTime = newTime;

        if(target != null) {
        current = path.calculate(deltaTime, current, target);
        m_drivetrain.setControl(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(current.speeds)
        );
        }
    }


    @Override
    public boolean isFinished() {
        if(target==null || path == null) return true;
        double targetdist = m_drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation());
        return targetdist <0.1;
    }


}



