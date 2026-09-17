package frc.robot.subsystems;


import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeShoot;
import frc.robot.constants.TransferConstants;


public class Transfer extends SubsystemBase {
    private final TalonFX m_transferMotor1 = new TalonFX(TransferConstants.kTransferId1, TransferConstants.kTransferCANBus);


    private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);




    private final BooleanSubscriber m_HeadingLockedSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("Aim/HeadingLocked")
        .subscribe(false);
   
    private final DoubleSubscriber m_flywheelVelSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getDoubleTopic("Flywheel/ActualRPS")
        .subscribe(0.0);


    private final BooleanSubscriber m_atDesiredRPSSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("Flywheel/AtDesiredRPS")
        .subscribe(false);


    public Transfer() {
        m_transferMotor1.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        // ParentDevice.optimizeBusUtilizationForAll(m_floorMotor, m_transferMotor1, m_transferMotor2);
    }


    @Override
    public void periodic() {

    }


    public void runMotors(double transferVel) {
        m_transferMotor1.setControl(m_transferRequest.withVelocity(transferVel));
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
        return Commands.run(() -> runMotors(TransferConstants.kTransferDefaultVel), this);
    }


    public Command stopTransfer() {
        return Commands.run(() -> runMotors(0.0), this);
    }


    public Command reverseCommand() {
        return Commands.run(() -> runMotors(-TransferConstants.kTransferShootVel), this);
    }


    public Command shootCommand(boolean checkAim) {
        SmartDashboard.putBoolean("ShootCommandCheckAim", true);
        return Commands.sequence(
            new WaitCommand(0.05),
            Commands.run(() -> this.runMotors(0.0), this)
                .until(() -> ((m_HeadingLockedSub.get() && m_atDesiredRPSSub.get())) || m_flywheelVelSub.get() > 50.0 || !checkAim)
                .andThen(
                    Commands.run(() -> this.runMotors(TransferConstants.kTransferShootVel), this)
                ).finallyDo(() -> this.runMotors(0.0))
        );
    }


    public Command shootTimeCommand() {
        return Commands.run(() -> this.runMotors(TransferConstants.kTransferShootVel), this)
            .withTimeout(TransferConstants.kTransferShootTime)
            .finallyDo(() -> this.runMotors(0.0));
    }
}

