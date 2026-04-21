package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    // Values identified from your system identification
    public static final double kFlywheelkV = 0.125; 
    public static final double kFlywheelkA = 1.1357132862; 
    public static final double kFlywheelkS = 3.7;  

    // Tuning weights: 1.0 RPS error tolerance, 12.5V max effort
    public static final double kLQRVelocityTolerance = 1.0; 
    public static final double kLQRControlEffortVolts = 12.5;

    public static final int kTopLeftFlywheelId = 28;
    public static final int kTopRightFlywheelId = 26;
    public static final int kBottomLeftFlywheelId = 29;
    public static final int kBottomRightFlywheelId = 27;
    public static final int kHoodMotorId = 25;
    public static final CANBus kCANBus = new CANBus("rio");

    public static final double kMaxFlywheelRPS = 50.0;
    public static final double kPassingDormantVel = 15.0;
    public static final double kZoneDormantVel = 26.0;

    public static final double kPosAtMinAngle = 0.066162109375;
    public static final double kPosAtMaxAngle = 0.0;
    public static final double kMinAngle = 56.0;
    public static final double kMaxAngle = 90.0;
    public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

    public static final double kYFlywheelVel = 30.0;
    public static final double kYHoodAngle = 62.0;

    public static final double kManualHoodInc = 2.0;
    public static final double kManualFlywheelInc = 1.0;
    public static final double kFlywheelToleranceRPS = 1.0;

    public static final InterpolatingDoubleTreeMap kDistanceToRPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToRPS.put(0.0, 20.0);

        
        kDistanceToRPS.put(1.8309003982663195, 27.104368845791093);
        kDistanceToRPS.put(1.8309003982663195, 27.104368845791093);
        kDistanceToRPS.put(1.8309003982663195, 27.104368845791093);
        kDistanceToRPS.put(1.8309003982663195, 27.104368845791093);

        kDistanceToAngle.put(0.0, 62.0);
        kDistanceToAngle.put(999.0, 62.0);

        kDistanceToTOF.put(0.0, 0.0);
        kDistanceToTOF.put(999.0, 0.0);
    }

    public static TalonFXConfiguration getFlywheelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 40.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = kFlywheelkS;


        configs.CurrentLimits.SupplyCurrentLimit = 40.0;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

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

        configs.CurrentLimits.SupplyCurrentLimit = 10.0;

        configs.MotionMagic.MotionMagicAcceleration = 3.0;
        configs.MotionMagic.MotionMagicCruiseVelocity = 0.7;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }
}