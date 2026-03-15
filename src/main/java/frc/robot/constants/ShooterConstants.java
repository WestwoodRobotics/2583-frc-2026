package frc.robot.constants;

import java.util.TreeMap;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.utils.ShotParam;

public class ShooterConstants {
    public static final int kHoodMotorId = 24;
    public static final int kTopLeftFlywheelId = 27;
    public static final int kTopRightFlywheelId = 25;
    public static final int kBottomLeftFlywheelId = 28;
    public static final int kBottomRightFlywheelId = 26;
    public static final CANBus kCANBus = new CANBus("SwerveCAN");

    public static final double kMaxFlywheelRPS = 70.0;

    public static final double kPosAtMinAngle = 7.048828125;
    public static final double kPosAtMaxAngle = 0.0;
    public static final double kMinAngle = 54.0;
    public static final double kMaxAngle = 90.0;
    public static final double kPerDegree = (kPosAtMaxAngle - kPosAtMinAngle) / (kMaxAngle - kMinAngle);

    public static final double kTrueMinAngle = 6.12890625;

    public static final double kManualHoodVolts = 1.0;
    public static final double kManualFlywheelInc = 1.0;
    
    public static final double kAutoBumperFlywheelVel = 40.83051602354075;
    public static final double kAutoBumperHoodAngle = 57.0;
    public static final double kFlywheelToleranceRPS = 2.5;

    public static final TreeMap<Double, ShotParam> kDistanceToShotParam = new TreeMap<>();

    public static final InterpolatingDoubleTreeMap kDistanceToTOF = new InterpolatingDoubleTreeMap();

    static {
        kDistanceToShotParam.put(0.0, new ShotParam(90.0, 42.89518492140559));
        kDistanceToShotParam.put(3.3300660570256118, new ShotParam(53.937655860349125, 47.23629288970057));
        kDistanceToShotParam.put(3.122054251971471, new ShotParam(60.204488778054866,47.23629288970057));
        kDistanceToShotParam.put(20.0, new ShotParam(54.61532834580216, kMaxFlywheelRPS));
        kDistanceToShotParam.put(2.4222653738038153, new ShotParam(55.43640897755611, 42.89518492140559));
        kDistanceToShotParam.put(1.0264647912083584, new ShotParam(71.15211970074813,39.479756897617726));
        kDistanceToShotParam.put(0.8573516665381802, new ShotParam(74.81795511221945,41.033560356574846));
        kDistanceToShotParam.put(1.178746466841953, new ShotParam(64.00503740648379,40.43880109318974));
        kDistanceToShotParam.put(1.5174186249342343, new ShotParam(57.48628428927681,40.43880109318974));
        kDistanceToShotParam.put(1.8774186249342343, new ShotParam(56.723192019950126,42.93880109318974));
        kDistanceToShotParam.put(1.9859927899645144, new ShotParam(55.52369077306733,41.26808332176154));
        kDistanceToShotParam.put(2.0986322962872097, new ShotParam(56.593516209476306,42.91862054667378));
        kDistanceToShotParam.put(2.2494403074332383, new ShotParam(56.55860349127182,43.91862054667378));
        kDistanceToShotParam.put(2.6003618518219382, new ShotParam(55.83042394014963,44.91292192907958));
        kDistanceToShotParam.put(3.2407349541380075, new ShotParam(66.35411471321696,55.83051602354075));
        kDistanceToShotParam.put(11.0, new ShotParam(55.83042394014963, kMaxFlywheelRPS));

        kDistanceToShotParam.put(2.5691115830316966, new ShotParam(58.65336658354115,46.55503599455011));
        kDistanceToShotParam.put(1.596343114508182, new ShotParam(63.53865336658354,42.55503599455011));

        kDistanceToTOF.put(0.0, 0.0);
        kDistanceToTOF.put(999.0, 0.0);
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

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.5;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        configs.CurrentLimits.StatorCurrentLimit = 40.0;
        configs.CurrentLimits.SupplyCurrentLimit = 30.0;
        
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        return configs;
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