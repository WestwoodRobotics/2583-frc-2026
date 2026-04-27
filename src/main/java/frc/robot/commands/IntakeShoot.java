package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.utils.GetTargetLocation;

public class IntakeShoot extends Command {
    
    private Intake m_intake;
    private CommandXboxController m_controller;

    public IntakeShoot(Intake intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("intakeshoot", true);
        m_intake.slowPivot();
        m_intake.setRollerVelocity(0.0);
    }

    @Override
    public void execute() {
        if (m_controller.leftTrigger().getAsBoolean()) {
            m_intake.setPivotPosition(IntakeConstants.kPivotOut);
            m_intake.setRollerVelocity(IntakeConstants.kRollerIntakingVel);
        }
        else {
            if (!GetTargetLocation.inZone()) {
                m_intake.setPivotPosition(IntakeConstants.kPivotOut);
                m_intake.setRollerVelocity(0.0);
                return;
            }
            m_intake.setPivotPosition(IntakeConstants.kPivotShoot);
            double m_newPos = m_intake.getPivotPosition();
            if (m_newPos > IntakeConstants.kPivotShoot - IntakeConstants.kJammedThreshold) {
                m_intake.setRollerVelocity(IntakeConstants.kRollerShootingVel);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.normalPivot();
        m_intake.setPivotPosition(IntakeConstants.kPivotOut);
    }
}
