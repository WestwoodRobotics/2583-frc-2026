package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeShoot;
import frc.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX m_floorMotor = new TalonFX(TransferConstants.kFloorId, TransferConstants.kFloorCANBus);
    private final TalonFX m_transferMotor1 = new TalonFX(TransferConstants.kTransferId1, TransferConstants.kTransferCANBus);
    private final TalonFX m_transferMotor2 = new TalonFX(TransferConstants.kTransferId2, TransferConstants.kTransferCANBus);

    private final VelocityTorqueCurrentFOC m_floorRequest = new VelocityTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);

    private final Follower m_transferFollower = new Follower(TransferConstants.kTransferId1, MotorAlignmentValue.Aligned);

    private final BooleanSubscriber m_headingLockedSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("HeadingLocked")
        .subscribe(true);

    private final BooleanSubscriber m_atDesiredRPSSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("Flywheel/AtDesiredRPS")
        .subscribe(true);

    public Transfer() {
        m_floorMotor.getConfigurator().apply(TransferConstants.getFloorMotorConfigs());
        m_transferMotor1.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        m_transferMotor2.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        // ParentDevice.optimizeBusUtilizationForAll(m_floorMotor, m_transferMotor1, m_transferMotor2);
    }

    @Override
    public void periodic() {
        m_transferMotor2.setControl(m_transferFollower);
    }

    public void runMotors(double floorVel, double transferVel) {
        m_floorMotor.setControl(m_floorRequest.withVelocity(floorVel));
        m_transferMotor1.setControl(m_transferRequest.withVelocity(transferVel));
        m_transferMotor2.setControl(m_transferFollower);
    }

    /**
     * Gets the current velocity of the primary transfer motor.
     * @return Velocity in rotations per second
     */
    public double getTransferVelocity() {
        return m_transferMotor1.getVelocity().getValueAsDouble();
    }

    public Command defaultCommand() {
        // Default: Floor spins at default speed
        return Commands.run(() -> runMotors(TransferConstants.kFloorDefaultVel, TransferConstants.kTransferDefaultVel), this);
    }

    public Command stopTransfer() {
        return Commands.run(() -> runMotors(0.0, 0.0), this);
    }

    public Command reverseCommand() {
        return Commands.run(() -> runMotors(-TransferConstants.kFloorShootVel, -TransferConstants.kTransferShootVel), this);
    }

    public Command shootCommand(boolean checkAim) {
        return Commands.run(() -> this.runMotors(0.0, 0.0), this)
            .until(() -> (m_headingLockedSub.get() && m_atDesiredRPSSub.get()) || !checkAim)
            .andThen(
                Commands.run(() -> this.runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel), this)
            ).finallyDo(() -> this.runMotors(0.0, 0.0));
    }

    public Command shootTimeCommand() {
        return Commands.run(() -> this.runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel), this)
            .withTimeout(TransferConstants.kTransferShootTime)
            .finallyDo(() -> this.runMotors(0.0, 0.0));
    }
}