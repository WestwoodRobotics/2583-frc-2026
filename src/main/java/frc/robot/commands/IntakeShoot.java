package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeShoot extends Command {
    
    private Intake m_intake;
    private CommandXboxController m_controller;

    private Timer m_intervalTimer = new Timer();

    public IntakeShoot(Intake intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("intakeshoot", true);
        m_intervalTimer.restart();
    }

    @Override
    public void execute() {
        if (m_controller.leftTrigger().getAsBoolean()) {
            m_intake.setPivotPosition(IntakeConstants.kPivotOut);
            m_intake.setRollerVelocity(IntakeConstants.kRollerIntakingVel);
        }
        else {
            m_intake.setPivotPosition(IntakeConstants.kPivotShoot);
            if (m_intake.getPivotPosition() >= IntakeConstants.kPivotShoot - 0.03) {
                m_intake.setRollerVelocity(IntakeConstants.kRollerShootingVel);
            } else {
                m_intake.setRollerVelocity(0.0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setPivotPosition(IntakeConstants.kPivotOut);
        m_intervalTimer.stop();
    }
}
