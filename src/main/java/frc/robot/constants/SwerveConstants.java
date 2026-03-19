package frc.robot.constants;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveConstants {
    public static final double fieldWidth = 16.513048;
    public static final double fieldLength = 8.042656;
    public static final double allianceZoneWidth = 4.611624;


    public static final Transform2d robotToShooter = new Transform2d(
        new Translation2d(0.1344549, 0.0),
        new Rotation2d(Math.toRadians(0))
    );


    public static final Translation2d blueHub = new Translation2d(allianceZoneWidth, fieldLength / 2);
    public static final Translation2d redHub = new Translation2d(fieldWidth - allianceZoneWidth, fieldLength / 2);


    public static final double upperCornerY = 7.350;
    public static final double lowerCornerY = fieldLength - upperCornerY;
    public static final double blueCornerX = 4;
    public static final double redCornerX = fieldWidth - blueCornerX;


    public static final double upperBlueCornerAngle = Math.toRadians(-79.58837132);
    public static final double lowerBlueCornerAngle = Math.toRadians(79.58837132);
    public static final double upperRedCornerAngle = Math.toRadians(79.58837132 - 180);
    public static final double lowerRedCornerAngle = Math.toRadians(-79.58837132 + 180);


    public static final double alignMaxVel = 3;
    public static final double alignMaxAccel = 3;
    public static final double alignMaxOmega = 1.5;
    public static final double alignMaxAlpha = 2;


    public static final double alignLatency = 0.1;


    public static final double bluePassX = allianceZoneWidth;
    public static final double redPassX = fieldWidth - bluePassX;
    public static final double lowerPassY = 2.0;
    public static final double upperPassY = fieldLength - lowerPassY;


    public static final double aimKp = 9.0;
    public static final double aimKi = 0.0;
    public static final double aimKd = 0.5;


    public static final double kResetX = 3.560;
    public static final double kResetY = 4.029;
}

