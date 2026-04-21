package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final CANBus canBus = ShooterConstants.kCANBus;
    private final TalonFX m_hoodMotor = new TalonFX(ShooterConstants.kHoodMotorId, canBus);
    private final TalonFX m_bottomLeftFlywheel = new TalonFX(ShooterConstants.kBottomLeftFlywheelId, canBus);
    private final TalonFX m_bottomRightFlywheel = new TalonFX(ShooterConstants.kBottomRightFlywheelId, canBus);
    private final TalonFX m_topLeftFlywheel = new TalonFX(ShooterConstants.kTopLeftFlywheelId, canBus);
    private final TalonFX m_topRightFlywheel = new TalonFX(ShooterConstants.kTopRightFlywheelId, canBus);

    // LQR Plant and Controller (Manual implementation for zero filtering)
    private final LinearSystem<N1, N1, N1> m_flywheelPlant = 
        LinearSystemId.identifyVelocitySystem(ShooterConstants.kFlywheelkV, ShooterConstants.kFlywheelkA);

    private final LinearQuadraticRegulator<N1, N1, N1> m_lqr = 
        new LinearQuadraticRegulator<>(m_flywheelPlant, 
        VecBuilder.fill(ShooterConstants.kLQRVelocityTolerance), 
        VecBuilder.fill(ShooterConstants.kLQRControlEffortVolts), 0.020);

    private final VoltageOut m_flywheelVoltageRequest = new VoltageOut(0.0);
    private final MotionMagicTorqueCurrentFOC m_hoodRequest = new MotionMagicTorqueCurrentFOC(0.0);

    private final Follower m_alignedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Aligned);
    private final Follower m_opposedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Opposed);

    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Shooter");
    private final DoublePublisher m_flywheelDesiredRPS = m_table.getDoubleTopic("Flywheel/DesiredRPS").publish();
    private final DoublePublisher m_flywheelIncerceptDelta = m_table.getDoubleTopic("Flywheel/InterceptDelta").publish();
    // private final DoublePublisher m_flywheelActualRPS = m_table.getDoubleTopic("Flywheel/ActualRPS").publish();
    private final BooleanPublisher m_atDesiredRPS = m_table.getBooleanTopic("Flywheel/AtDesiredRPS").publish();
    // private final DoublePublisher m_hoodDesiredPos = m_table.getDoubleTopic("Hood/DesiredPos").publish();
    private final DoublePublisher m_hoodActualPos = m_table.getDoubleTopic("Hood/ActualPos").publish();
    private final DoublePublisher m_hoodDesiredAngle = m_table.getDoubleTopic("Hood/DesiredAngle").publish();
    private final DoublePublisher m_hoodActualAngle = m_table.getDoubleTopic("Hood/ActualAngle").publish();
    private final BooleanPublisher m_autoAimEnabledPub = m_table.getBooleanTopic("AutoAimEnabled").publish();
    private final BooleanPublisher m_isManualPub = m_table.getBooleanTopic("FlywheelOn").publish();
    private final BooleanPublisher m_dormantModePub = m_table.getBooleanTopic("DormantModeOn").publish();

    private double m_targetRPS = 0.0;
    private double m_desiredAngle = ShooterConstants.kMinAngle;
    private boolean m_autoAimEnabled = false;
    private boolean m_hoodUp = false;
    private boolean m_dormantMode = true;
    public double m_intercept = 19.39717;
    public double m_delta = 0;

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

        // Flywheel LQR Control Logic
        if (m_targetRPS > 0.5) {
            var u_lqr = m_lqr.calculate(VecBuilder.fill(flywheelVel), VecBuilder.fill(m_targetRPS));
            double feedforwardVoltage = (ShooterConstants.kFlywheelkV * m_targetRPS) + (Math.signum(m_targetRPS) * ShooterConstants.kFlywheelkS);
            double totalVoltage = u_lqr.get(0, 0) + feedforwardVoltage;
    
            m_topRightFlywheel.setControl(m_flywheelVoltageRequest.withOutput(totalVoltage));
        } else {
            m_topRightFlywheel.setControl(new NeutralOut());
            m_lqr.reset();
        }

        // Follower Logic
        m_bottomRightFlywheel.setControl(m_alignedFollower);
        m_topLeftFlywheel.setControl(m_opposedFollower);
        m_bottomLeftFlywheel.setControl(m_opposedFollower);
        
        m_flywheelDesiredRPS.set(m_targetRPS);
        m_atDesiredRPS.set(Math.abs(flywheelVel - m_targetRPS) < ShooterConstants.kFlywheelToleranceRPS);

        m_hoodActualPos.set(hoodPos);
        m_hoodDesiredAngle.set(m_desiredAngle);
        double actualAngle = (hoodPos - ShooterConstants.kPosAtMinAngle) / ShooterConstants.kPerDegree + ShooterConstants.kMinAngle;
        m_hoodActualAngle.set(actualAngle);

        m_autoAimEnabledPub.set(m_autoAimEnabled);
        m_isManualPub.set(!m_autoAimEnabled);
        m_dormantModePub.set(m_dormantMode);
        m_flywheelIncerceptDelta.set(m_delta);
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

    public void changeHoodAngle(double delta) {
        m_autoAimEnabled = false;
        double newPos = m_desiredAngle + delta;
        newPos = MathUtil.clamp(newPos, ShooterConstants.kMinAngle, ShooterConstants.kMaxAngle);
        setHoodAngle(newPos);
    }

    public void changeFlywheelVelocity(double delta) {
        m_autoAimEnabled = false;
        double newVel = m_targetRPS + delta;
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

    public void toogleDormantMode() {
        m_dormantMode = !m_dormantMode;
    }

    public boolean getDormantMode() {
        return m_dormantMode;
    }

    public void changeIntercept(double delta) {
        m_delta += delta;
        m_intercept += delta;
    }

    public void setFlywheelVelocity(double velocity) {
        m_targetRPS = MathUtil.clamp(velocity, 0.0, ShooterConstants.kMaxFlywheelRPS);
    }
}