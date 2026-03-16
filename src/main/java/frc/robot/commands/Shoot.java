package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TransferConstants;
import frc.robot.subsystems.Transfer;

public class Shoot extends Command {
    
    private Transfer m_transfer;
    private boolean m_isJammed;
    private Timer m_jamTimer = new Timer();
    private Timer m_reverseTimer = new Timer();

    private final BooleanSubscriber m_canShootSub = NetworkTableInstance.getDefault()
        .getTable("Shooter")
        .getBooleanTopic("CanShoot")
        .subscribe(false);

    public Shoot(Transfer transfer) {
        m_transfer = transfer;
        addRequirements(m_transfer);
    }

    @Override
    public void initialize() {
        m_isJammed = false;
        m_jamTimer.restart();
        m_reverseTimer.stop();
        m_reverseTimer.reset();
    }

    @Override
    public void execute() {
        // Inside autonomous, only allow shooting if the robot is aligned (canShoot is true)
        if (DriverStation.isAutonomous() && !m_canShootSub.get()) {
            m_transfer.runMotors(0.0, 0.0);
            return;
        }

        if (m_isJammed) {
            // Run in reverse to clear the jam
            m_transfer.runMotors(-TransferConstants.kFloorShootVel, -TransferConstants.kTransferShootVel);
            
            if (m_reverseTimer.hasElapsed(TransferConstants.kReverseTimeSecs)) {
                m_isJammed = false;
                m_jamTimer.restart();
            }
        } else {
            // Normal shooting
            m_transfer.runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel);
            
            // Check if the velocity is below the jam threshold
            if (Math.abs(m_transfer.getTransferVelocity()) < TransferConstants.kJamThreshold) {
                if (m_jamTimer.hasElapsed(TransferConstants.kJamTimeSecs)) {
                    m_isJammed = true;
                    m_reverseTimer.restart();
                }
            } else {
                m_jamTimer.restart(); // Reset the timer if we are above the threshold
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_transfer.runMotors(0.0, 0.0);
    }
}