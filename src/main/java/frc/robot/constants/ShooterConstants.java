package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final int kTopLeftFlywheelId = 27;
    public static final int kTopRightFlywheelId = 25;
    public static final int kBottomLeftFlywheelId = 28;
    public static final int kBottomRightFlywheelId = 26;
    public static final CANBus kCANBus = new CANBus("SwerveCAN");

    public static final double kMaxFlywheelRPS = 70.0;

    public static final double kManualFlywheelInc = 1.0;
    
    public static final double kAutoBumperFlywheelVel = 51.84;
    public static final double kFlywheelToleranceRPS = 2.5;

    public static final InterpolatingDoubleTreeMap kDistanceToRPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToRPS.put(0.0, 32.47639);

        kDistanceToRPS.put(7.225356, 70.0);
       
        kDistanceToRPS.put(Double.MAX_VALUE, kMaxFlywheelRPS);

        kDistanceToTOF.put(0.0, 0.0);
        kDistanceToTOF.put(999.0, 0.0);
    }

    public static TalonFXConfiguration getFlywheelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        
        configs.Slot0.kP = 5.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 2.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.2;

        configs.CurrentLimits.StatorCurrentLimit = 40.0;

        configs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;

        configs.Feedback.SensorToMechanismRatio = (42.0 / 16.0) * (15.0 / 33.0);
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        return configs;
    }
}