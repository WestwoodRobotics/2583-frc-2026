package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId, IntakeConstants.kCANBus);
    private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId, IntakeConstants.kCANBus);

    private final MotionMagicTorqueCurrentFOC m_pivotRequest = new MotionMagicTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC m_rollerRequest = new VelocityTorqueCurrentFOC(0.0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final SysIdRoutine m_pivotSysIdRoutine = 
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

    private final SysIdRoutine m_rollerSysIdRoutine  = 
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        state -> SignalLogger.writeString("SysIdRoller_state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                        (volts) -> m_rollerMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null,
                        this)
                );

    private SysIdRoutine m_sysIdRoutineToApply = m_rollerSysIdRoutine;

    private final NetworkTable m_intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    private final DoublePublisher m_pivotDesiredPub = m_intakeTable.getDoubleTopic("Pivot/DesiredPos").publish();
    private final DoublePublisher m_pivotActualPub = m_intakeTable.getDoubleTopic("Pivot/ActualPos").publish();
    private final DoublePublisher m_rollerDesiredPub = m_intakeTable.getDoubleTopic("Roller/DesiredVelocityRPS").publish();
    private final DoublePublisher m_rollerActualPub = m_intakeTable.getDoubleTopic("Roller/ActualVelocityRPS").publish();

    public Intake() {
        // Apply configurations directly from constants to keep constructor clean of variables
        m_pivotMotor.getConfigurator().apply(IntakeConstants.getPivotConfigs());
        m_rollerMotor.getConfigurator().apply(IntakeConstants.getRollerConfigs());

        // Apply pivot offset
        m_pivotMotor.setPosition(IntakeConstants.kPivotOffset);

        m_pivotMotor.setControl(m_pivotRequest.withPosition(IntakeConstants.pivotIn));
    }

    @Override
    public void periodic() {
        double pivotActual = m_pivotMotor.getPosition().getValueAsDouble();
        double rollerActual = m_rollerMotor.getVelocity().getValueAsDouble();

        m_pivotDesiredPub.set(m_pivotRequest.Position);
        m_pivotActualPub.set(pivotActual);
        m_rollerDesiredPub.set(m_rollerRequest.Velocity);
        m_rollerActualPub.set(rollerActual);
    }

    /**
     * Sets the position of the pivot motor using Motion Magic Expo (Torque Current FOC).
     * @param position Target position in rotations.
     */
    public void setPivotPosition(double position) {
        m_pivotMotor.setControl(m_pivotRequest.withPosition(position));
    }

    /**
     * Sets the velocity of the roller motor using Motion Magic Velocity (Torque Current FOC).
     * @param velocity Target velocity in rotations per second.
     */
    public void setRollerVelocity(double velocity) {
        m_rollerMotor.setControl(m_rollerRequest.withVelocity(velocity));
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

    public Command fullRetract() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotIn), this);
    }

    public Command partialRetract() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.pivotPartial), this);
    }

    public Command resetPivot() {
        return Commands.runOnce(() -> m_pivotMotor.setPosition(IntakeConstants.pivotOut + 0.1), this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
}
