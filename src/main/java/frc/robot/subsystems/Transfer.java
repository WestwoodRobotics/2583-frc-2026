package frc.robot.subsystems;

<<<<<<< HEAD
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX floorMotor = new TalonFX(TransferConstants.kFloorId, new CANBus(TransferConstants.kCANBus));
    private final TalonFX transferMotor = new TalonFX(TransferConstants.kTransferId, new CANBus(TransferConstants.kCANBus));

    private final MotionMagicVelocityTorqueCurrentFOC floorRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final MotionMagicVelocityTorqueCurrentFOC transferRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    public Transfer() {
        floorMotor.getConfigurator().apply(TransferConstants.getVelocityMotorConfigs());
        transferMotor.getConfigurator().apply(TransferConstants.getVelocityMotorConfigs());
    }

    private void runMotors(double floorVel, double transferVel) {
        floorMotor.setControl(floorRequest.withVelocity(floorVel));
        transferMotor.setControl(transferRequest.withVelocity(transferVel));
    }

    public Command defaultCommand() {
        // Default: Floor spins at default speed, Transfer is stopped (default speed is 0.0 in constants)
        return run(() -> runMotors(TransferConstants.kFloorDefaultVel, TransferConstants.kTransferDefaultVel));
    }

    public Command intakeCommand() {
        // Left Trigger: Floor spins at intake speed, Transfer remains at default (stopped)
        return run(() -> runMotors(TransferConstants.kFloorIntakeVel, TransferConstants.kTransferDefaultVel));
    }

    public Command shootCommand() {
        // Right Trigger: Both motors spin at their respective shooting velocities
        return run(() -> runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel));
    }
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase{
    
>>>>>>> 5e5a5e9 (rebase constants)
}
