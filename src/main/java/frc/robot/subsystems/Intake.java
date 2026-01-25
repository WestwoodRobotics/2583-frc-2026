package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX m_positionMotor;
    private final TalonFX m_velocityMotor;

    private final MotionMagicExpoTorqueCurrentFOC m_expoRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    private double pivotPos = IntakeConstants.inPos;

    public Intake() {
        m_positionMotor = new TalonFX(IntakeConstants.kPositionMotorId, IntakeConstants.kCANBus);
        m_velocityMotor = new TalonFX(IntakeConstants.kVelocityMotorId, IntakeConstants.kCANBus);

        // Apply configurations directly from constants to keep constructor clean of variables
        m_positionMotor.getConfigurator().apply(IntakeConstants.getPositionMotorConfigs());
        m_velocityMotor.getConfigurator().apply(IntakeConstants.getVelocityMotorConfigs());
    }

    /**
     * Sets the position of the pivot motor using Motion Magic Expo (Torque Current FOC).
     * @param position Target position in rotations.
     */
    public void setPosition(double position) {
        pivotPos = position;
        m_positionMotor.setControl(m_expoRequest.withPosition(pivotPos));
    }

    /**
     * Get the desired position of the pivot motor.
     * @return Desired position in rotations.
     */
    public double getPosition() {
        return pivotPos;
    }

    /**
     * Sets the velocity of the roller motor using Motion Magic Velocity (Torque Current FOC).
     * @param velocity Target velocity in rotations per second.
     */
    public void setVelocity(double velocity) {
        m_velocityMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }

    /**
     * Toggle the position of the pivot motor.
     * @return Command to toggle the position of the pivot motor.
     */
    public Command TogglePivot() {
        return Commands.runOnce(() -> {
            pivotPos = this.getPosition() == IntakeConstants.inPos ? IntakeConstants.outPos : IntakeConstants.inPos;
            this.setPosition(pivotPos);
        }, this);
    }
}
