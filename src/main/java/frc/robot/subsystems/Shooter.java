package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.ShotParam;

public class Shooter extends SubsystemBase {

    private final TalonFX m_hoodMotor;
    private final TalonFX m_bottomLeftFlywheel;
    private final TalonFX m_bottomRightFlywheel;
    private final TalonFX m_topLeftFlywheel;
    private final TalonFX m_topRightFlywheel;

    private final MotionMagicExpoTorqueCurrentFOC m_expoRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private SysIdRoutine m_hoodSysIdRoutine;
    private SysIdRoutine m_flywheelSysIdRoutine;
    private SysIdRoutine m_routineToApply;

    private final Follower m_alignedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Aligned);
    private final Follower m_opposedFollower = new Follower(ShooterConstants.kTopRightFlywheelId, MotorAlignmentValue.Opposed);

    public Shooter() {
        CANBus canBus = ShooterConstants.kCANBus;
        m_hoodMotor = new TalonFX(ShooterConstants.kHoodMotorId, canBus);
        m_bottomLeftFlywheel = new TalonFX(ShooterConstants.kBottomLeftFlywheelId, canBus);
        m_bottomRightFlywheel = new TalonFX(ShooterConstants.kBottomRightFlywheelId, canBus);
        m_topLeftFlywheel = new TalonFX(ShooterConstants.kTopLeftFlywheelId, canBus);
        m_topRightFlywheel = new TalonFX(ShooterConstants.kTopRightFlywheelId, canBus);

        // Apply configurations directly from constants to keep constructor clean of variables
        m_hoodMotor.getConfigurator().apply(ShooterConstants.getHoodMotorConfigs());
        m_bottomLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_bottomRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topLeftFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());
        m_topRightFlywheel.getConfigurator().apply(ShooterConstants.getFlywheelMotorConfigs());

        m_hoodSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(1),
                null,
                state -> SignalLogger.writeString("SysIdHood_state", state.toString())
            
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> m_hoodMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this
            )
        );

        m_flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> SignalLogger.writeString("SysIdFlywheel_state", state.toString())
            
            ), 
            new SysIdRoutine.Mechanism(
                (volts) -> m_hoodMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                null,
                this
            )
        );

        m_routineToApply = m_hoodSysIdRoutine;
    }

    public void setHoodPosition(double position) {
        m_hoodMotor.setControl(m_expoRequest.withPosition(position));
    }

    public void setHoodAngle(double angle) {
        double clampedAngle = Math.clamp(angle, ShooterConstants.minAngle, ShooterConstants.maxAngle);
        double angleDelta = clampedAngle - ShooterConstants.minAngle;
        double position = ShooterConstants.posAtMinAngle + angleDelta * ShooterConstants.perDegree;

        setHoodPosition(position);
    }

    public void setFlywheelVelocity(double velocity) {
        m_topRightFlywheel.setControl(m_velocityRequest.withVelocity(velocity));
        m_bottomRightFlywheel.setControl(m_opposedFollower);
        m_topLeftFlywheel.setControl(m_opposedFollower);
        m_bottomLeftFlywheel.setControl(m_alignedFollower);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }

    public ShotParam getShotParam(double distance) {
        var map = ShooterConstants.kDistanceToShotParam;
        Map.Entry<Double, ShotParam> floor = map.floorEntry(distance);
        Map.Entry<Double, ShotParam> ceil = map.ceilingEntry(distance);

        if (floor == null && ceil == null) return new ShotParam(ShooterConstants.minAngle, 0);
        if (floor == null) return ceil.getValue();
        if (ceil == null) return floor.getValue();

        ShotParam floorVal = floor.getValue();
        ShotParam ceilVal = ceil.getValue();

        if (floor.getKey().equals(ceil.getKey())) return floorVal;

        double t = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
        double angle = floorVal.angle + t * (ceilVal.angle - floorVal.angle);
        double velocity = floorVal.velocity + t * (ceilVal.velocity - floorVal.velocity);

        angle = Math.clamp(angle, ShooterConstants.minAngle, ShooterConstants.maxAngle);
        return new ShotParam(angle, velocity);
    }
}
