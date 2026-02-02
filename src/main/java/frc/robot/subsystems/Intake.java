package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX m_pivotMotor;
    private final TalonFX m_rollerMotor;

    private final MotionMagicExpoTorqueCurrentFOC m_expoRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_pivotSysIdRoutine;
    private final SysIdRoutine m_rollerSysIdRoutine;

    private SysIdRoutine m_sysIdRoutineToApply;

    public Intake() {
        m_pivotMotor = new TalonFX(IntakeConstants.kPositionMotorId, IntakeConstants.kCANBus);
        m_rollerMotor = new TalonFX(IntakeConstants.kVelocityMotorId, IntakeConstants.kCANBus);

        // Apply configurations directly from constants to keep constructor clean of variables
        m_pivotMotor.getConfigurator().apply(IntakeConstants.getPositionMotorConfigs());
        m_rollerMotor.getConfigurator().apply(IntakeConstants.getVelocityMotorConfigs());

        m_pivotSysIdRoutine = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(0.25).per(Second),
                    Volts.of(1),
                    null,
                    state -> SignalLogger.writeString("SysIdPivot_state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> m_pivotMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                    null,
                    this)
            );
        
        m_rollerSysIdRoutine  = 
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                        Volts.of(0.25).per(Second),
                        Volts.of(1),
                        null,
                        state -> SignalLogger.writeString("SysIdRoller_state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                        (volts) -> m_pivotMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this)
                );

        m_sysIdRoutineToApply = m_pivotSysIdRoutine;
    }

    /**
     * Sets the position of the pivot motor using Motion Magic Expo (Torque Current FOC).
     * @param position Target position in rotations.
     */
    public void setPivotPosition(double position) {
        m_pivotMotor.setControl(m_expoRequest.withPosition(position));
    }

    /**
     * Sets the velocity of the roller motor using Motion Magic Velocity (Torque Current FOC).
     * @param velocity Target velocity in rotations per second.
     */
    public void setRollerVelocity(double velocity) {
        m_rollerMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }

    public Command intakeDefault() {
        return Commands.run(
            () -> setRollerVelocity(IntakeConstants.rollerNeutralVel),
            this
        );
    }

    // Set position to out and velocity to intaking
    public Command runIntake() {
        return Commands.run(() -> {
            setPivotPosition(IntakeConstants.pivotOut);
            setRollerVelocity(IntakeConstants.rollerIntakingVel);
        }, this);
    }

    public Command retractIntake() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotIn), this);
    }

    public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
}
