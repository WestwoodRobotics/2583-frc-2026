package frc.robot.commands;


import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;


import org.photonvision.PhotonCamera;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
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


    private Vision vision;
    private PhotonCamera camera;




    private Timer timer;
    private double currentTime;
    private double deltaTime;


    public DriveToObject(CommandSwerveDrivetrain drivetrain, Vision vision){
        m_drivetrain = drivetrain;
        timer = new Timer();
        this.vision = vision;
        camera = vision.getCamera(5);
    }


    @Override
    public void initialize(){

        targets = new ArrayList<Pose2d>();
        nears = new ArrayList<Pose2d>();
        List<Transform3d> transforms = vision.getObjectPos(camera, VisionConstants.robotToCamTransforms[2]);


        for(int i = 0; i < transforms.size(); i++){
            Transform3d object3d = transforms.get(i);


            Transform2d object2d = new Transform2d(
                object3d.getX(),
                object3d.getY(),
                object3d.getRotation().toRotation2d()
            );


            targets.add(m_drivetrain.getState().Pose.plus(object2d));
        }

        if(targets.isEmpty()) return;
       
        Iterator<Pose2d> iterator = targets.iterator();


        for(int j = 0; j < targets.size(); j++){


            Pose2d object = iterator.next();
            Translation2d objTranslation = object.getTranslation();

            List<Pose2d> noObj = new ArrayList<>(targets);
            noObj.remove(object);

            if(noObj.isEmpty()) continue;

            Pose2d near = object.nearest(noObj);
            Translation2d nearTranslation = near.getTranslation();


            Double distance = objTranslation.getDistance(nearTranslation);
            if(distance < 2 && !nears.contains(object)){
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


        path = new LinearPath(
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxVel, SwerveConstants.alignMaxAccel),
            new TrapezoidProfile.Constraints(SwerveConstants.alignMaxOmega, SwerveConstants.alignMaxAlpha)
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
        return path!= null && path.isFinished(currentTime);
    }


}



