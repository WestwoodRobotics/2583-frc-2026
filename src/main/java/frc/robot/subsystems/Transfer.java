package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX m_floorMotor = new TalonFX(TransferConstants.kFloorId, TransferConstants.kFloorCANBus);
    private final TalonFX m_transferMotor1 = new TalonFX(TransferConstants.kTransferId1, TransferConstants.kTransferCANBus);
    private final TalonFX m_transferMotor2 = new TalonFX(TransferConstants.kTransferId2, TransferConstants.kTransferCANBus);

    private final VelocityTorqueCurrentFOC m_floorRequest = new VelocityTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);
    private final TorqueCurrentFOC m_torqueReq = new TorqueCurrentFOC(0.0);

    private final Follower m_transferInvertedFollower = new Follower(TransferConstants.kTransferId1, MotorAlignmentValue.Opposed);

    private final BooleanSubscriber m_canShootSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("CanShoot")
        .subscribe(true);

    private final SysIdRoutine m_floorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(5.0).per(Second),
            Volts.of(30.0),
            null,
            (state) -> SignalLogger.writeString("floor_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_floorMotor.setControl(m_torqueReq.withOutput(volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_transferSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(5.0).per(Second),
            Volts.of(30.0),
            null,
            (state) -> SignalLogger.writeString("transfer_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_transferMotor1.setControl(m_torqueReq.withOutput(volts.in(Volts)));
                m_transferMotor2.setControl(m_torqueReq.withOutput(-volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_routineToApply = m_transferSysIdRoutine;

    public Transfer() {
        m_floorMotor.getConfigurator().apply(TransferConstants.getFloorMotorConfigs());
        m_transferMotor1.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        m_transferMotor2.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());

        ParentDevice.optimizeBusUtilizationForAll(m_floorMotor, m_transferMotor1, m_transferMotor2);
    }

    public void runMotors(double floorVel, double transferVel) {
        m_floorMotor.setControl(m_floorRequest.withVelocity(floorVel));
        m_transferMotor1.setControl(m_transferRequest.withVelocity(transferVel));
        m_transferMotor2.setControl(m_transferInvertedFollower);
    }

    /**
     * Gets the current velocity of the primary transfer motor.
     * @return Velocity in rotations per second
     */
    public double getTransferVelocity() {
        return m_transferMotor1.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic(){}

    public Command defaultCommand() {
        // Default: Floor spins at default speed
        double transferSpeed = DriverStation.isAutonomous() ? 0.0 : TransferConstants.kTransferDefaultVel;
        return Commands.run(() -> runMotors(TransferConstants.kFloorDefaultVel, transferSpeed), this);
    }

    public Command reverseCommand() {
        return Commands.run(() -> runMotors(-TransferConstants.kFloorShootVel, -TransferConstants.kTransferShootVel), this);
    }

    public Command shootCommand() {
        return Commands.run(() -> {
            if (DriverStation.isAutonomous() && !m_canShootSub.get()) {
            this.runMotors(0.0, 0.0);
            return;
        }
        this.runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel);
        }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }
}