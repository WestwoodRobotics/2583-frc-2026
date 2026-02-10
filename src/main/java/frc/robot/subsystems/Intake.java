package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
    // Tracks whether the intake command is actively running. Exposed as a supplier
    // so external systems (like FuelSim) can query it without depending on
    // command scheduling internals.
    private volatile boolean m_intakeRunning = false;
    public BooleanSupplier intakerun = () -> m_intakeRunning;
    private final MotionMagicExpoTorqueCurrentFOC m_expoRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

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
    public void setPivotPosition(double position) {
        m_positionMotor.setControl(m_expoRequest.withPosition(position));
    }

    /**
     * Sets the velocity of the roller motor using Motion Magic Velocity (Torque Current FOC).
     * @param velocity Target velocity in rotations per second.
     */
    public void setRollerVelocity(double velocity) {
        m_velocityMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }

    public Command intakeDefault() {
        return Commands.run(
            () -> setRollerVelocity(IntakeConstants.rollerNeutralVel),
            this
        );
    }

    // Set position to out and velocity to intaking
    public Command runIntake() {
        return Commands.runEnd(
            () -> {
                // active behavior
                setPivotPosition(IntakeConstants.pivotOut);
                setRollerVelocity(IntakeConstants.rollerIntakingVel);
                m_intakeRunning = true;
            },
            () -> {
                // cleanup on end
                m_intakeRunning = false;
            },
            this
        ).withTimeout(2.0);
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotIn), this);
    }
}