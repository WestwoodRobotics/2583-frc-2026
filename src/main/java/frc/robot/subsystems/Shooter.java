package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final CANBus canBus = ShooterConstants.kCANBus;
    private final TalonFX m_bottomLeftFlywheel = new TalonFX(ShooterConstants.kBottomLeftFlywheelId, canBus);
    private final TalonFX m_bottomRightFlywheel = new TalonFX(ShooterConstants.kBottomRightFlywheelId, canBus);
    private final TalonFX m_topLeftFlywheel = new TalonFX(ShooterConstants.kTopLeftFlywheelId, canBus);
    private final TalonFX m_topRightFlywheel = new TalonFX(ShooterConstants.kTopRightFlywheelId, canBus);

    private final VelocityTorqueCurrentFOC m_flywheelRequest = new VelocityTorqueCurrentFOC(0.0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private SysIdRoutine m_flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> SignalLogger.writeString("SysIdFlywheel_state", state.toString())
            
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    m_topRightFlywheel.setControl(m_voltReq.withOutput(volts.in(Volts)));
                    m_bottomRightFlywheel.setControl(m_voltReq.withOutput(volts.in(Volts)));
                    m_topLeftFlywheel.setControl(m_voltReq.withOutput(-volts.in(Volts)));
                    m_bottomLeftFlywheel.setControl(m_voltReq.withOutput(-volts.in(Volts)));
                },
                null,
                this
            )
        );
    
    private final SysIdRoutine m_routineToApply = m_flywheelSysIdRoutine;

    private final Follower m_alignedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Aligned);
    private final Follower m_opposedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Opposed);

    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shooter");
    // private final DoublePublisher m_flywheelDesiredRPS = m_table.getDoubleTopic("Flywheel/DesiredRPS").publish();
    // private final DoublePublisher m_flywheelActualRPS = m_table.getDoubleTopic("Flywheel/ActualRPS").publish();
    // private final BooleanPublisher m_atDesiredRPS = m_table.getBooleanTopic("Flywheel/AtDesiredRPS").publish();
    private final BooleanPublisher m_autoAimEnabledPub = m_table.getBooleanTopic("AutoAimEnabled").publish();
    private final BooleanPublisher m_isManualPub = m_table.getBooleanTopic("FlywheelOn").publish();

    private boolean m_autoAimEnabled = true;

    public Shooter() {
        // Apply configurations directly from constants to keep constructor clean of variables
        m_bottomLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_bottomRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());

    }

    @Override
    public void periodic() {
        double flywheelVel = m_topRightFlywheel.getVelocity().getValueAsDouble();

        // m_flywheelDesiredRPS.set(m_flywheelRequest.Velocity);
        // m_flywheelActualRPS.set(flywheelVel);
        // m_atDesiredRPS.set(Math.abs(flywheelVel - m_flywheelRequest.Velocity) < ShooterConstants.kFlywheelToleranceRPS);

        m_autoAimEnabledPub.set(m_autoAimEnabled);
        m_isManualPub.set(!m_autoAimEnabled);
        SignalLogger.writeDouble("Shooter/FlywheelActualVel", flywheelVel);
        SignalLogger.writeDouble("Shooter/FlywheelDesiredVel", m_flywheelRequest.Velocity);

        SignalLogger.writeDouble("Shooter/FlywheelBLCurrent", this.m_bottomLeftFlywheel.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Shooter/FlywheelBRCurrent", this.m_bottomRightFlywheel.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Shooter/FlywheelTLCurrent", this.m_topLeftFlywheel.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Shooter/FlywheelTRCurrent", this.m_topRightFlywheel.getSupplyCurrent().getValueAsDouble());
    }

    public void toggleAutoAim() {
        m_autoAimEnabled = !m_autoAimEnabled;
    }

    public void setAutoAim(boolean aim) {
        m_autoAimEnabled = aim;
        if (!aim) {
            setFlywheelVelocity(0.0);
        }
    }

    public boolean isAutoAimEnabled() {
        return m_autoAimEnabled;
    }

    public void changeFlywheelVelocity(double delta) {
        m_autoAimEnabled = false;
        double newVel = m_flywheelRequest.Velocity + delta;
        newVel = MathUtil.clamp(newVel, 0.0, ShooterConstants.kMaxFlywheelRPS);
        setFlywheelVelocity(newVel);
    }

    public void setFlywheelVelocity(double velocity) {
        m_topRightFlywheel.setControl(m_flywheelRequest.withVelocity(velocity));
        m_bottomRightFlywheel.setControl(m_alignedFollower);
        m_topLeftFlywheel.setControl(m_opposedFollower);
        m_bottomLeftFlywheel.setControl(m_opposedFollower);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }
}
