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

    public static final double kMaxFlywheelRPS = 82.1666666667;

    public static final double kPosAtMinAngle = 7.048828125;
    public static final double kPosAtMaxAngle = 0.0;
    public static final double kMinAngle = 57.0;
    public static final double kMaxAngle = 90.0;
    public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

    public static final double kTrenchFlywheelVel = 48.99627892532494;
    public static final double kBumperHoodAngle = 57.0;
    public static final double kBumperFlywheelVel = 45.0;

    public static final double kManualHoodTorque = 1.0;
    public static final double kManualFlywheelInc = 1.0;
    public static final double kFlywheelToleranceRPS = 1.0;

    public static final InterpolatingDoubleTreeMap kDistanceToRPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToRPS.put(0.0, 32.47639);
        kDistanceToRPS.put(7.225356, 70.0);
        kDistanceToRPS.put(Double.MAX_VALUE, kMaxFlywheelRPS);

        kDistanceToAngle.put(0.0, 90.0);
        kDistanceToAngle.put(999.0, kMinAngle);

        kDistanceToTOF.put(0.0, 0.0);
        kDistanceToTOF.put(999.0, 0.0);
    }

    public static TalonFXConfiguration getFlywheelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 5.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 7.2;
        configs.Slot0.kV = 0.512;
        configs.Slot0.kA = 5.32797793656;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        configs.Feedback.SensorToMechanismRatio = 1.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        return configs;
    }

    public static TalonFXConfiguration getHoodMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 20.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.4;
        configs.Slot0.kS = 3.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.2;
        configs.Slot0.kG = 0.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        return configs;
    }
}