package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final int kTopLeftFlywheelId = 19;
    public static final int kTopRightFlywheelId = 20;
    public static final int kHoodMotorId = 18;
    public static final int kTurretMotorId = 17;



    public static final double kTurretMinAngle = -200.0;
    public static final double kTurretMaxAngle = 200.0;

    public static final double kTurretWrapDeadbandDegrees = 5;
    public static final double kTurretAcceptanceDeg = 10;

    public static double kTurretOffset = 0.0;


    public static final CANBus kCANBus = new CANBus("rio");

    public static final double kMaxFlywheelRPS = 65.0;
    public static final double kPassingDormantVel = 0.0;
    public static final double kZoneDormantVel = 0.0;

    public static final double kPosAtMinAngle = 0.035;
    public static final double kPosAtMaxAngle = 0.0;
    public static final double kMinAngle = 68.0;
    public static final double kMaxAngle = 90.0;
    public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

    public static final double kTrueMinAngle = 68.0;

    public static final double kYFlywheelVel = 27.858921832551054;
    public static final double kYHoodAngle = 60.0;

    public static final double kHoodWrapDeadbandDegrees = 5.0;
    public static final double kManualHoodInc = 2.0;
    public static final double kManualFlywheelInc = 1.0;
    public static final double kFlywheelToleranceRPS = 2.0;

    public static final double kPeakReverseCurrentLimit = -5.0;

    public static final InterpolatingDoubleTreeMap kDistanceToRPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    public static final double SOTMVelMultiplier = 0.410022779043;
    static {
        

        kDistanceToRPS.put(999.0, 60.0);
        kDistanceToRPS.put( 2.855596979495011 ,  53.9921875);
        kDistanceToRPS.put( 1.3968957512393252 ,          46.9921875);
        kDistanceToRPS.put( 2.0522009506120975 ,          51.9921875);
        kDistanceToRPS.put( 2.8279531120872687,          48.9921875);
        kDistanceToRPS.put( 3.3487300066087062,      51.99516125819079);
        kDistanceToRPS.put( 3.7220687003550847,      53.997413746738545);
        kDistanceToRPS.put(         4.13497477067495,      52.99989047003573);





        kDistanceToAngle.put( 4.13497477067495,      71.12444196428572);
        kDistanceToAngle.put( 3.7220687003550847,      77.109375);
        kDistanceToAngle.put(3.3487300066087062, 77.87667410714286);
        kDistanceToAngle.put(2.8279531120872687, 79.71819196428572);
        kDistanceToAngle.put(2.0522009506120975, 85.08928571428572);
        kDistanceToAngle.put(1.3968957512393252, 82.78738839285714);
        kDistanceToAngle.put(999.0, 60.0);

        

        kDistanceToTOF.put(0.0, 0.97);
        kDistanceToTOF.put(1.27156941954409, 0.97);
        kDistanceToTOF.put(1.6964201919252078, 1.23);
        kDistanceToTOF.put(2.0143223838950965, 1.24);
        kDistanceToTOF.put(2.444507623684114, 1.42);
        kDistanceToTOF.put(999.0, 1.42);
    }

    public static TalonFXConfiguration getFlywheelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 10.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 10.0;
        configs.Slot0.kV = 0.099;
        configs.Slot0.kA = 0.1;

        // configs.CurrentLimits.SupplyCurrentLimit = 40.0;

        configs.Feedback.SensorToMechanismRatio = 1.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getHoodMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 780.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 20.0;
        configs.Slot0.kS = 5.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 2.203;
        configs.Slot0.kG = 2.0;

        configs.Feedback.SensorToMechanismRatio = (38.0 /12.0) * (50.0 / 1.0) * (29.0 / 30.0); //NOT ACCURATE RN

        configs.CurrentLimits.SupplyCurrentLimit = 11.0;

        configs.MotionMagic.MotionMagicAcceleration = 2.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getTurretMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 27.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 3.0;
        configs.Slot0.kS = 1.8;
        configs.Slot0.kV = 0.0526315789474;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kG = 0.0;

        // configs.Feedback.RotorToSensorRatio = 132.0;
        configs.Feedback.SensorToMechanismRatio = (87.0/2.0);
        // configs.Feedback.FeedbackRemoteSensorID = 999; // CANcoder ID
        // configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;


        configs.CurrentLimits.SupplyCurrentLimit = 11.0;

        configs.MotionMagic.MotionMagicAcceleration = 3.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 2.1;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (kTurretMaxAngle / 360.0);
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (kTurretMinAngle / 360.0);
        
        return configs;
    }

    public double[] robotDistancesInterpolatingArray = {0, 1, 2, 3, 4};
    public double[] flywheelVelocitiesInterpolatingArray = {0, 10, 20, 30, 40};

}