package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

    public static final class LEDConstants {
        public static final int candleId = 30;
        public static final String canBus = "CANivore";
        public static final int endIndex = 26;
    }

    public static final class SwerveConstants {

        public static final double fieldWidth = 16.540988;
        public static final double fieldLength = 8.069326;
        public static final double allianceZoneWidth = 4.625594;

        public static final Transform2d shooterToRobot = new Transform2d(
            new Translation2d(0.0, 0.0),
            new Rotation2d(Math.toRadians(0))
        );

        public static final Translation2d blueHub = new Translation2d(allianceZoneWidth, fieldLength / 2);
        public static final Translation2d redHub = new Translation2d(fieldWidth - allianceZoneWidth, fieldLength / 2);

        public static final Pose2d leftTower = new Pose2d(2.0, 5.0, new Rotation2d());
        public static final double alignMaxVel = 1.0;
        public static final double alignMaxAccel = 1.0;
        public static final double alignMaxOmega = 0.75;
        public static final double alignMaxAlpha = 0.75;

        public static final double aimKp = 4.0;
        public static final double aimKi = 0.0;
        public static final double aimKd = 0.1;
    }
}
