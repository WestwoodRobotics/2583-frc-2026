package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class LEDConstants {
    public static final int candleId = 40;
    public static final CANBus canBus = new CANBus("rio");
    public static final int endIndex = 49;

    public static final double kMaxHeadingError = 5.0;
    public static final double kRumbleIntensity = 0.7;
    public static final double kRumbleTimeout = 0.2;
}