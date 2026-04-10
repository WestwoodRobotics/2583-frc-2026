package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeWiggle extends Command {
    
    private Intake m_intake;
    private CommandXboxController m_controller;

    private Timer m_intervalTimer = new Timer();

    public IntakeWiggle(Intake intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intervalTimer.restart();
    }

    @Override
    public void execute() {
        if (m_controller.leftTrigger().getAsBoolean()) {
            m_intake.setPivotPosition(IntakeConstants.kPivotOut);
            m_intake.setRollerVelocity(IntakeConstants.kRollerIntakingVel);
        }
        else {
            if(m_intervalTimer.hasElapsed(IntakeConstants.kWiggleTime)) {
                m_intake.setPivotPosition(IntakeConstants.kPivotShoot);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPivotPosition(IntakeConstants.kPivotOut);
        m_intervalTimer.stop();
    }
}
