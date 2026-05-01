package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final int kTopLeftFlywheelId = 28;
    public static final int kTopRightFlywheelId = 26;
    public static final int kBottomLeftFlywheelId = 29;
    public static final int kBottomRightFlywheelId = 27;
    public static final int kHoodMotorId = 25;
    public static final CANBus kCANBus = new CANBus("rio");

    public static final double kMaxFlywheelRPS = 60.0;
    public static final double kPassingDormantVel = 0.0;
    public static final double kZoneDormantVel = 0.0;

    public static final double kPosAtMinAngle = 0.066162109375;
    public static final double kPosAtMaxAngle = 0.0;
    public static final double kMinAngle = 56.0;
    public static final double kMaxAngle = 90.0;
    public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

    public static final double kTrueMinAngle = 45.0;

    public static final double kYFlywheelVel = 27.858921832551054;
    public static final double kYHoodAngle = 60.0;

    public static final double kManualHoodInc = 2.0;
    public static final double kManualFlywheelInc = 1.0;
    public static final double kFlywheelToleranceRPS = 2.0;

    public static final double kPeakReverseCurrentLimit = -5.0;

    public static final InterpolatingDoubleTreeMap kDistanceToRPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    static {

        kDistanceToRPS.put(0.0, 25.763912905171967);
        kDistanceToRPS.put(1.1062981283038664, 25.763912905171967);
        kDistanceToRPS.put(1.31226468083407, 26.80544394808964);
        kDistanceToRPS.put(1.7317843665102108, 27.858921832551054);
        kDistanceToRPS.put(1.9877911771679004, 29.14);
        kDistanceToRPS.put(2.401767735037994, 32.208838675189966);
        kDistanceToRPS.put(2.9683119546531347, 35.541559773265675);
        kDistanceToRPS.put(6.81583, 60.0);
        kDistanceToRPS.put(999.0, 60.0);

        kDistanceToAngle.put(0.0, 70.0);
        kDistanceToAngle.put(1.1062981283038664, 70.0);
        kDistanceToAngle.put(1.31226468083407, 66.0);
        kDistanceToAngle.put(1.7317843665102108, 60.0);
        kDistanceToAngle.put(1.9877911771679004, 60.0);
        kDistanceToAngle.put(2.401767735037994, 60.0);
        kDistanceToAngle.put(2.9683119546531347, 60.0);
        kDistanceToAngle.put(999.0, 60.0);

        kDistanceToTOF.put(0.0, 0.0);
        kDistanceToTOF.put(999.0, 0.0);
    }

    public static TalonFXConfiguration getFlywheelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 10.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.1;
        configs.Slot0.kS = 2.1;
        configs.Slot0.kV = 0.0920634921;
        configs.Slot0.kA = 1.0781533433;

        // configs.CurrentLimits.SupplyCurrentLimit = 40.0;

        configs.Feedback.SensorToMechanismRatio = 1.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getHoodMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 1000.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 50.0;
        configs.Slot0.kS = 5.8;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 3.203;
        configs.Slot0.kG = 2.3;

        configs.Feedback.SensorToMechanismRatio = (54.0 / 11.0) * (28.0 / 15.0) * (106.0 / 7.0);

        configs.CurrentLimits.SupplyCurrentLimit = 11.0;

        configs.MotionMagic.MotionMagicAcceleration = 3.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 0.7;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}