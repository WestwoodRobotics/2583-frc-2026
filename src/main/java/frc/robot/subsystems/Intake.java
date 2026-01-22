package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX m_positionMotor;
    private final TalonFX m_velocityMotor;

    private final MotionMagicExpoTorqueCurrentFOC m_expoRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    private double m_targetPosition = 0;
    private int m_activeSlot = 0;

    public Intake() {
        m_positionMotor = new TalonFX(IntakeConstants.kPositionMotorId, IntakeConstants.kCANBus);
        m_velocityMotor = new TalonFX(IntakeConstants.kVelocityMotorId, IntakeConstants.kCANBus);

        // Apply configurations directly from constants to keep constructor clean of variables
        m_positionMotor.getConfigurator().apply(IntakeConstants.getPositionMotorConfigs());
        m_velocityMotor.getConfigurator().apply(IntakeConstants.getVelocityMotorConfigs());
    }

    @Override
    public void periodic() {
        // Monitor the closed loop error to detect excessive force/collisions
        double currentError = Math.abs(m_positionMotor.getClosedLoopError().getValueAsDouble());

        if (m_activeSlot == 0 && currentError > IntakeConstants.collisionErrorThreshold) {
            // Switch to Slot 1 (Dampened/Lower Gains)
            m_activeSlot = 1;
            m_positionMotor.setControl(m_expoRequest.withPosition(m_targetPosition).withSlot(m_activeSlot));
        } 
        else if (m_activeSlot == 1 && currentError < IntakeConstants.recoveryErrorThreshold) {
            // Switch back to Slot 0 (Normal Gains)
            m_activeSlot = 0;
            m_positionMotor.setControl(m_expoRequest.withPosition(m_targetPosition).withSlot(m_activeSlot));
        }
    }

    /**
     * Sets the position of the first motor using Motion Magic Expo (Torque Current FOC).
     * @param position Target position in rotations.
     */
    public void setPosition(double position) {
        m_targetPosition = position;
        m_positionMotor.setControl(m_expoRequest.withPosition(m_targetPosition).withSlot(m_activeSlot));
    }

    /**
     * Sets the velocity of the second motor using Motion Magic Velocity (Torque Current FOC).
     * @param velocity Target velocity in rotations per second.
     */
    public void setVelocity(double velocity) {
        m_velocityMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }
}
