package frc.robot.subsystems;

<<<<<<< HEAD
<<<<<<< HEAD
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX m_floorMotor = new TalonFX(TransferConstants.kFloorId, new CANBus(TransferConstants.kCANBus));
    private final TalonFX m_transferMotor = new TalonFX(TransferConstants.kTransferId, new CANBus(TransferConstants.kCANBus));

    private final MotionMagicVelocityTorqueCurrentFOC m_floorRequest = new MotionMagicVelocityTorqueCurrentFOC(0);
    private final MotionMagicVelocityTorqueCurrentFOC m_transferRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_floorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("floor_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> m_floorMotor.setControl(m_voltReq.withOutput(volts.in(Volts))), 
            null, 
            this)
    );

    private final SysIdRoutine m_transferSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("floor_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> m_transferMotor.setControl(m_voltReq.withOutput(volts.in(Volts))), 
            null, 
            this)
    );

    private final SysIdRoutine m_routineToApply = m_floorSysIdRoutine;

    public Transfer() {
        m_floorMotor.getConfigurator().apply(TransferConstants.getVelocityMotorConfigs());
        m_transferMotor.getConfigurator().apply(TransferConstants.getVelocityMotorConfigs());
    }

    private void runMotors(double floorVel, double transferVel) {
        m_floorMotor.setControl(m_floorRequest.withVelocity(floorVel));
        m_transferMotor.setControl(m_transferRequest.withVelocity(transferVel));
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }
=======
=======
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
>>>>>>> 614d92b (rebased constant imports)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

<<<<<<< HEAD
public class Transfer extends SubsystemBase{
    
>>>>>>> 14ae7a6 (rebased constants)
=======
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
>>>>>>> 614d92b (rebased constant imports)
}
