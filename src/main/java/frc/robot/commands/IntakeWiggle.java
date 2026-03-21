package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeWiggle extends Command {
    
    private Intake m_intake;

    private Timer m_intervalTimer = new Timer();

    public IntakeWiggle(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intervalTimer.restart();
        m_intake.setRollerVelocity(IntakeConstants.kRollerShootingVel);
    }

    @Override
    public void execute() {
        if (m_intervalTimer.get() < IntakeConstants.kWiggleTime) {
            m_intake.setPivotPosition(IntakeConstants.kPivotShoot);
        } else if (m_intervalTimer.get() < IntakeConstants.kWiggleTime * 2) {
            m_intake.setPivotPosition(IntakeConstants.kPivotOut);
        } else {
            m_intervalTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPivotPosition(IntakeConstants.kPivotShoot);
        m_intervalTimer.stop();
    }
}
