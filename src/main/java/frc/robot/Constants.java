package frc.robot;

public class Constants {

    public static final class LEDConstants {
        public static final int candleId = 30;
        public static final String canBus = "CANivore";
        public static final int endIndex = 26;
    }

    public static final class TransferConstants {
        public static final int kFloorId = 1;
        public static final int kTransferId = 2;

        public static final double kFloorDefaultVel = 0.0;
        public static final double kFloorIntakeVel = 0.0;
        public static final double kFloorShootVel = 0.0;

        public static final double kTransferDefaultVel = 0.0;
        public static final double kTransferShootVel = 0.0;
    }
}
