package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
    public static final class IntakeConstants {
        public static final int kPositionMotorId = 1;
        public static final int kVelocityMotorId = 2;
        public static final CANBus kCANBus = new CANBus("rio");

        public static final double pivotIn = 0.0;
        public static final double pivotOut = 0.1;

        public static final double rollerNeutralVel = 0.0;
        public static final double rollerIntakingVel = 0.0;
        public static final double rollerShootingVel = 0.0;

        public static TalonFXConfiguration getPositionMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;
            configs.Slot0.kG = 0.0;
            configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            
            configs.MotionMagic.MotionMagicExpo_kV = 0.0;
            configs.MotionMagic.MotionMagicExpo_kA = 0.0;

            configs.CurrentLimits.StatorCurrentLimit = 0.0;
            configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }

        public static TalonFXConfiguration getVelocityMotorConfigs() {
            TalonFXConfiguration configs = new TalonFXConfiguration();
            
            configs.Slot0.kP = 0.0;
            configs.Slot0.kI = 0.0;
            configs.Slot0.kD = 0.0;
            configs.Slot0.kS = 0.0;
            configs.Slot0.kV = 0.0;
            configs.Slot0.kA = 0.0;

            configs.CurrentLimits.StatorCurrentLimit = 0.0;
            configs.CurrentLimits.SupplyCurrentLimit = 0.0;

            configs.Feedback.SensorToMechanismRatio = 0.0;
            
            configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            
            return configs;
        }
    }
}