package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX m_floorMotor1 = new TalonFX(TransferConstants.kFloorId1, new CANBus(TransferConstants.kCANBus));
    private final TalonFX m_transferMotor1 = new TalonFX(TransferConstants.kTransferId1, new CANBus(TransferConstants.kCANBus));
    private final TalonFX m_transferMotor2 = new TalonFX(TransferConstants.kTransferId2, new CANBus(TransferConstants.kCANBus));

    private final VelocityTorqueCurrentFOC m_floorRequest = new VelocityTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final Follower m_transferInvertedFollower = new Follower(TransferConstants.kTransferId1, MotorAlignmentValue.Opposed);

    private final SysIdRoutine m_floorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("floor_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_floorMotor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_transferSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("transfer_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_transferMotor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
                m_transferMotor2.setControl(m_voltReq.withOutput(-volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_routineToApply = m_transferSysIdRoutine;

    public Transfer() {
        m_floorMotor1.getConfigurator().apply(TransferConstants.getFloorMotorConfigs());
        m_transferMotor1.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        m_transferMotor2.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        SignalLogger.start();
    }

    public void runMotors(double floorVel, double transferVel) {
        m_floorMotor1.setControl(m_floorRequest.withVelocity(floorVel));
        m_transferMotor1.setControl(m_transferRequest.withVelocity(transferVel));
        m_transferMotor2.setControl(m_transferInvertedFollower);
    }

    @Override
    public void periodic(){
        SignalLogger.writeDouble("Transfer floor current", this.m_floorMotor1.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Transfer 1 current", this.m_transferMotor1.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Transfer 2 current", this.m_transferMotor2.getSupplyCurrent().getValueAsDouble());

    }

    public Command defaultCommand() {
        // Default: Floor spins at default speed, Transfer is stopped (default speed is 0.0 in constants)
        return Commands.run(() -> runMotors(TransferConstants.kFloorDefaultVel, TransferConstants.kTransferDefaultVel), this);
    }

    public Command shootCommand() {
        // Right Trigger: Both motors spin at their respective shooting velocities
        return Commands.run(() -> runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel), this);
    }

    public Command reverseCommand() {
        return Commands.run(() -> runMotors(-TransferConstants.kFloorShootVel, -TransferConstants.kTransferShootVel), this)
            .beforeStarting(() -> {
                TorqueCurrentConfigs config = new TorqueCurrentConfigs();
                config.PeakReverseTorqueCurrent = -800.0;
                config.PeakForwardTorqueCurrent = 0.0;
                m_transferMotor1.getConfigurator().apply(config);
                m_transferMotor2.getConfigurator().apply(config);
            })
            .finallyDo((interrupted) -> {
                TorqueCurrentConfigs config = new TorqueCurrentConfigs();
                config.PeakReverseTorqueCurrent = 0.0;
                config.PeakForwardTorqueCurrent = 800.0;
                m_transferMotor1.getConfigurator().apply(config);
                m_transferMotor2.getConfigurator().apply(config);
            });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }
}