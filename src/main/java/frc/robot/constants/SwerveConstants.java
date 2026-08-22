package frc.robot.constants;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveConstants {
    public static final double fieldWidth = 16.540988;
    public static final double fieldLength = 8.069326;
    public static final double allianceZoneWidth = 4.625594;


    public static final Transform2d robotToShooter = new Transform2d(
        new Translation2d(0.1905, 0.0),
        new Rotation2d(Math.toRadians(0))
    );
    public static final Transform2d robotToTurret = new Transform2d(
        new Translation2d(0.1905, 0.0),
        new Rotation2d(Math.toRadians(0))
    );


    public static final Translation2d blueHub = new Translation2d(allianceZoneWidth, fieldLength / 2);
    public static final Translation2d redHub = new Translation2d(fieldWidth - allianceZoneWidth, fieldLength / 2);


    public static final double bluePassX = 4.0;
    public static final double redPassX = fieldWidth - bluePassX;
    public static final double lowerPassY = 2.0;
    public static final double upperPassY = fieldLength - lowerPassY;


    public static final double aimKp = 9.8638;
    public static final double aimKi = 0.0;
    public static final double aimKd = 0.23757;


    public static final double kResetX = 3.560;
    public static final double kResetY = 4.029;


    public static final double kBrakeTime = 0.1;


    public static final double kDrivePower = 3.0;
    public static final double kSlowmodeK = 0.5;
}

