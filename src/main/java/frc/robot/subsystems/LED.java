package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.LEDConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.GetHubStatus;
import frc.robot.utils.GetTargetLocation;

public class LED extends SubsystemBase {

    private CANdle candle;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driver;

    private boolean wasHubActive;
    private boolean wasDisabled;
    private boolean strobing = false;

    private Color currentColor = new Color(0,0,0);

    private final NetworkTable m_hubStatusTable = NetworkTableInstance.getDefault().getTable("HubStatus");
    private final BooleanPublisher m_isHubActivePub = m_hubStatusTable.getBooleanTopic("IsHubActive").publish();
    private final DoublePublisher m_countdownPub = m_hubStatusTable.getDoubleTopic("countdown").publish();
    private final DoublePublisher m_endgamePub = m_hubStatusTable.getDoubleTopic("endgame").publish();
    private final BooleanPublisher m_isPracticePub = m_hubStatusTable.getBooleanTopic("IsPractice").publish();

    private final NetworkTable m_shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    private final BooleanPublisher m_headingLockedPub = m_shooterTable.getBooleanTopic("HeadingLocked").publish();

    public LED(CommandSwerveDrivetrain drivetrain, CommandXboxController driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        candle = new CANdle(LEDConstants.candleId, LEDConstants.canBus);

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        candle.getConfigurator().apply(cfg);

        wasHubActive = GetHubStatus.isHubActive();
        wasDisabled = DriverStation.isDisabled();
    }

    @Override
    public void periodic() {
        boolean isDisabled = DriverStation.isDisabled();

        if (!isDisabled && wasDisabled) {
            clearAllAnimations();
        }
        wasDisabled = isDisabled;

        boolean isHubActive = GetHubStatus.isHubActive();
        double countdown = GetHubStatus.getHubCountdown();
        m_isHubActivePub.set(isHubActive);
        m_countdownPub.set(countdown);
        m_endgamePub.set(GetHubStatus.getEndgameCountdown());
        m_isPracticePub.set(GetHubStatus.isPractice());

        Pose2d robotPose = drivetrain.getState().Pose;
        boolean isAligned = isAligned(robotPose);
        m_headingLockedPub.set(isAligned);

        if (isDisabled) {
            currentColor = new Color(255,60,0);
            startLarsonAnimation(currentColor);
            return;
        }

        if (countdown < 3) {
            strobing = true;
        } else {
            strobing = false;
        }

        if (isHubActive != wasHubActive) {
            CommandScheduler.getInstance().schedule(
                Commands.startEnd(
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, LEDConstants.kRumbleIntensity);
                },
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                }
            ).withTimeout(LEDConstants.kRumbleTimeout));
        }
        wasHubActive = isHubActive;

        if(DriverStation.getMatchTime() <= 30 && DriverStation.isTeleop()){
            startFireAnimation();
            return;
        }

        if (!isHubActive) {
            currentColor = Color.kRed;
            setSolidColor(Color.kRed);
            return;
        }

        if (isAligned) {
            currentColor = Color.kGreen;
            setSolidColor(currentColor);
            return;
        }

        if (strobing) {
            currentColor = Color.kPurple;
            setStrobeAnimation(currentColor);
            return;
        }
        currentColor = new Color(255,60,0);
        setSolidColor(currentColor);
     }

    private boolean isAligned(Pose2d robotPose) {
        Translation2d target = GetTargetLocation.getTargetLocation(robotPose, drivetrain.getState().Speeds);
        if (target == null) return false;

        Pose2d shooterPose = robotPose.plus(SwerveConstants.robotToShooter);
        Rotation2d angleToTarget = target.minus(shooterPose.getTranslation()).getAngle();
        Rotation2d requiredRobotRotation = angleToTarget.minus(SwerveConstants.robotToShooter.getRotation());
        
        double error = Math.abs(robotPose.getRotation().minus(requiredRobotRotation).getDegrees());
        return error <= LEDConstants.kMaxHeadingError;
    }

    public void setSolidColor(Color color, double brightness){
        if (strobing) return;
        candle.setControl(new SolidColor(8, LEDConstants.endIndex).withColor(new RGBWColor(color).scaleBrightness(brightness)));
    }

    public void setSolidColor(Color color){
        this.setSolidColor(color, 1);
    }
    
    public void clearColor(){
        candle.setControl(new SolidColor(8, LEDConstants.endIndex).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
    }
    
    public void startFireAnimation(){
        FireAnimation FIREforward = new FireAnimation(29, LEDConstants.endIndex).withBrightness(1).withCooling(0.25).withSparking(0.45).withFrameRate(40).withDirection(AnimationDirectionValue.Forward).withSlot(0);
        FireAnimation FIREbackward = new FireAnimation(8, 28).withBrightness(1).withCooling(0.25).withSparking(0.45).withFrameRate(40).withDirection(AnimationDirectionValue.Backward).withSlot(1);
        candle.setControl(FIREforward); 
        candle.setControl(FIREbackward);   
    }

    public void startLarsonAnimation(Color color){
        if (strobing) return;
        LarsonAnimation larson = new LarsonAnimation(8, LEDConstants.endIndex).withColor(new RGBWColor(color)).withSize(7).withFrameRate(45);
        candle.setControl(larson);   
    }

    public void setStrobeAnimation(Color color) {
        StrobeAnimation strobe = new StrobeAnimation(8, LEDConstants.endIndex).withColor(new RGBWColor(color)).withFrameRate(2);
        candle.setControl(strobe);
    }

    public void clearAllAnimations() {
        for (int slot = 0; slot < LEDConstants.endIndex; slot++) {
            candle.setControl(new EmptyAnimation(slot));
        }
    }
}