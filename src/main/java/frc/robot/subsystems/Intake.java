package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId, IntakeConstants.kCANBus);
    private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId, IntakeConstants.kCANBus);

    private final MotionMagicTorqueCurrentFOC m_pivotRequest = new MotionMagicTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC m_rollerRequest = new VelocityTorqueCurrentFOC(0.0);
    private final TorqueCurrentFOC m_torqueReq = new TorqueCurrentFOC(0.0);

    private final SysIdRoutine m_pivotSysIdRoutine = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(2.0).per(Second), // Represents 2 Amps per second
                    Volts.of(10.0),            // Represents a 10 Amp dynamic step
                    null,
                    state -> SignalLogger.writeString("SysIdPivot_state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) -> m_pivotMotor.setControl(m_torqueReq.withOutput(volts.in(Volts))),
                    null,
                    this)
            );

    private final SysIdRoutine m_rollerSysIdRoutine  = 
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                        Volts.of(5.0).per(Second),
                        Volts.of(30.0),
                        null,
                        state -> SignalLogger.writeString("SysIdRoller_state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                        (volts) -> m_rollerMotor.setControl(m_torqueReq.withOutput(volts.in(Volts))),
                        null,
                        this)
                );

    private SysIdRoutine m_sysIdRoutineToApply = m_rollerSysIdRoutine;

    private final NetworkTable m_intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
    // private final DoublePublisher m_pivotDesiredPub = m_intakeTable.getDoubleTopic("Pivot/DesiredPos").publish();
    private final DoublePublisher m_pivotActualPub = m_intakeTable.getDoubleTopic("Pivot/ActualPos").publish();
    // private final DoublePublisher m_rollerDesiredPub = m_intakeTable.getDoubleTopic("Roller/DesiredVelocityRPS").publish();
    // private final DoublePublisher m_rollerActualPub = m_intakeTable.getDoubleTopic("Roller/ActualVelocityRPS").publish();

    public Intake() {
        // Apply configurations directly from constants to keep constructor clean of variables
        m_pivotMotor.getConfigurator().apply(IntakeConstants.getPivotConfigs());
        m_rollerMotor.getConfigurator().apply(IntakeConstants.getRollerConfigs());

        // Apply pivot offset
        m_pivotMotor.setPosition(IntakeConstants.kPivotOffset);

        m_pivotMotor.setControl(m_pivotRequest.withPosition(IntakeConstants.kPivotIn));

        ParentDevice.optimizeBusUtilizationForAll(m_pivotMotor, m_rollerMotor);
    }

    @Override
    public void periodic() {
        double pivotActual = m_pivotMotor.getPosition().getValueAsDouble();
        // double rollerActual = m_rollerMotor.getVelocity().getValueAsDouble();

        // m_pivotDesiredPub.set(m_pivotRequest.Position);
        m_pivotActualPub.set(pivotActual);
        // m_rollerDesiredPub.set(m_rollerRequest.Velocity);
        // m_rollerActualPub.set(rollerActual);
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
            () -> setRollerVelocity(IntakeConstants.kRollerNeutralVel),
            this
        );
    }

    // Set position to out and velocity to intaking
    public Command runIntake() {
        return Commands.run(() -> {
            setPivotPosition(IntakeConstants.kPivotOut);
            setRollerVelocity(IntakeConstants.kRollerIntakingVel);
        }, this);
    }

    public Command fullRetract() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.kPivotIn), this);
    }

    public Command partialRetract() {
        return Commands.runOnce(() -> setPivotPosition(IntakeConstants.kPivotPartial), this);
    }

    public Command shootCommand() {
        return Commands.startEnd(() ->{
            setPivotPosition(IntakeConstants.kPivotShoot);
            setRollerVelocity(IntakeConstants.kRollerShootingVel);
        },
        () -> setPivotPosition(IntakeConstants.kPivotOut),
        this);
    }

    public void resetPivot() {
        m_pivotMotor.setPosition(IntakeConstants.kResetPivotPos);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
}
