package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    private final TalonFX m_hoodMotor = new TalonFX(ShooterConstants.kHoodMotorId, canBus);
    private final TalonFX m_bottomLeftFlywheel = new TalonFX(ShooterConstants.kBottomLeftFlywheelId, canBus);
    private final TalonFX m_bottomRightFlywheel = new TalonFX(ShooterConstants.kBottomRightFlywheelId, canBus);
    private final TalonFX m_topLeftFlywheel = new TalonFX(ShooterConstants.kTopLeftFlywheelId, canBus);
    private final TalonFX m_topRightFlywheel = new TalonFX(ShooterConstants.kTopRightFlywheelId, canBus);

    private final VelocityTorqueCurrentFOC m_flywheelRequest = new VelocityTorqueCurrentFOC(0.0);
    private final PositionTorqueCurrentFOC m_hoodRequest = new PositionTorqueCurrentFOC(0.0);
    private final TorqueCurrentFOC m_torqueReq = new TorqueCurrentFOC(0.0);

    private SysIdRoutine m_hoodSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second),
                Volts.of(10.0),
                null,
                state -> SignalLogger.writeString("SysIdHood_state", state.toString())
            
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> m_hoodMotor.setControl(m_torqueReq.withOutput(volts.in(Volts))),
                null,
                this
            )
        );

    private SysIdRoutine m_flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(5.0).per(Second),
                Volts.of(30.0),
                null,
                state -> SignalLogger.writeString("SysIdFlywheel_state", state.toString())
            
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    m_topRightFlywheel.setControl(m_torqueReq.withOutput(volts.in(Volts)));
                    m_bottomRightFlywheel.setControl(m_torqueReq.withOutput(volts.in(Volts)));
                    m_topLeftFlywheel.setControl(m_torqueReq.withOutput(-volts.in(Volts)));
                    m_bottomLeftFlywheel.setControl(m_torqueReq.withOutput(-volts.in(Volts)));
                },
                null,
                this
            )
        );
    
    private final SysIdRoutine m_routineToApply = m_flywheelSysIdRoutine;

    private final Follower m_alignedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Aligned);
    private final Follower m_opposedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Opposed);

    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shooter");
    private final DoublePublisher m_flywheelDesiredRPS = m_table.getDoubleTopic("Flywheel/DesiredRPS").publish();
    // private final DoublePublisher m_flywheelActualRPS = m_table.getDoubleTopic("Flywheel/ActualRPS").publish();
    private final BooleanPublisher m_atDesiredRPS = m_table.getBooleanTopic("Flywheel/AtDesiredRPS").publish();
    // private final DoublePublisher m_hoodDesiredPos = m_table.getDoubleTopic("Hood/DesiredPos").publish();
    // private final DoublePublisher m_hoodActualPos = m_table.getDoubleTopic("Hood/ActualPos").publish();
    // private final DoublePublisher m_hoodDesiredAngle = m_table.getDoubleTopic("Hood/DesiredAngle").publish();
    private final DoublePublisher m_hoodActualAngle = m_table.getDoubleTopic("Hood/ActualAngle").publish();
    private final BooleanPublisher m_autoAimEnabledPub = m_table.getBooleanTopic("AutoAimEnabled").publish();
    private final BooleanPublisher m_isManualPub = m_table.getBooleanTopic("FlywheelOn").publish();

    private double m_desiredAngle = ShooterConstants.kMinAngle;
    private boolean m_autoAimEnabled = true;
    private boolean m_hoodUp = false;

    public Shooter() {
        // Apply configurations directly from constants to keep constructor clean of variables
        m_hoodMotor.getConfigurator().apply(ShooterConstants.getHoodMotorConfigs());
        m_bottomLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_bottomRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());

    }

    @Override
    public void periodic() {
        double hoodPos = m_hoodMotor.getPosition().getValueAsDouble();
        double flywheelVel = m_topRightFlywheel.getVelocity().getValueAsDouble();
        // m_flywheelDesiredRPS.set(m_flywheelRequest.Velocity);
        // m_flywheelActualRPS.set(flywheelVel);
        // m_hoodDesiredPos.set(m_hoodRequest.Position);
        // m_hoodActualPos.set(hoodPos);
        // m_hoodDesiredAngle.set(m_desiredAngle);
        m_atDesiredRPS.set(Math.abs(flywheelVel - m_flywheelRequest.Velocity) < ShooterConstants.kFlywheelToleranceRPS);

        double actualAngle = (hoodPos - ShooterConstants.kPosAtMinAngle) / ShooterConstants.kPerDegree + ShooterConstants.kMinAngle;
        m_hoodActualAngle.set(actualAngle);

        m_autoAimEnabledPub.set(m_autoAimEnabled);
        m_isManualPub.set(!m_autoAimEnabled);
    }

    public void toggleAutoAim() {
        m_autoAimEnabled = !m_autoAimEnabled;
        if (!m_autoAimEnabled) {
            setFlywheelVelocity(0.0);
            setHoodAngle(90.0);
        }
    }

    public void setAutoAim(boolean aim) {
        m_autoAimEnabled = aim;
        if (!aim) {
            setFlywheelVelocity(0.0);
            setHoodAngle(90.0);
        }
    }

    public boolean isAutoAimEnabled() {
        return m_autoAimEnabled;
    }

    public void setHoodTorque(double torque) {
        m_autoAimEnabled = false;
        m_hoodMotor.setControl(m_torqueReq.withOutput(torque));
    }

    public void changeFlywheelVelocity(double delta) {
        m_autoAimEnabled = false;
        double newVel = m_flywheelRequest.Velocity + delta;
        newVel = MathUtil.clamp(newVel, 0.0, ShooterConstants.kMaxFlywheelRPS);
        setFlywheelVelocity(newVel);
    }

    public void setHoodPosition(double position) {
        m_hoodMotor.setControl(m_hoodRequest.withPosition(position));
    }

    public void setHoodAngle(double angle) {
        m_desiredAngle = MathUtil.clamp(angle, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
        double angleDelta = m_desiredAngle - ShooterConstants.kMinAngle;
        double position = ShooterConstants.kPosAtMinAngle + angleDelta * ShooterConstants.kPerDegree;

        setHoodPosition(position);
    }

    public void resetHoodPosition() {
        m_hoodMotor.setPosition(0.0);
    }

    public void setHood(boolean up) {
        m_hoodUp = up;
    }

    public boolean getHoodState() {
        return m_hoodUp;
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
