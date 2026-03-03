package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.GetHubStatus;
import frc.robot.utils.GetTargetLocation;

public class LED extends SubsystemBase {

    private CANdle candle;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driver;
    private final CommandXboxController operator;

    private boolean wasHubActive;

    private final NetworkTable m_hubStatusTable = NetworkTableInstance.getDefault().getTable("HubStatus");
    private final BooleanPublisher m_isHubActivePub = m_hubStatusTable.getBooleanTopic("IsHubActive").publish();
    private final DoublePublisher m_countdownPub = m_hubStatusTable.getDoubleTopic("countdown").publish();
    private final BooleanPublisher m_isPracticePub = m_hubStatusTable.getBooleanTopic("IsPractice").publish();

    private final NetworkTable m_shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
    private final BooleanPublisher m_canShootPub = m_shooterTable.getBooleanTopic("CanShoot").publish();

    public LED(CommandSwerveDrivetrain drivetrain, CommandXboxController driver, CommandXboxController operator) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.operator = operator;
        candle = new CANdle(LEDConstants.candleId, LEDConstants.canBus);

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.StripType = StripTypeValue.BRG;

        candle.getConfigurator().apply(cfg);

        wasHubActive = GetHubStatus.isHubActive();
    }

    @Override
    public void periodic() {
        boolean isHubActive = GetHubStatus.isHubActive();
        m_isHubActivePub.set(isHubActive);
        m_countdownPub.set(GetHubStatus.getHubCountdown());
        m_isPracticePub.set(GetHubStatus.isPractice());

        if (isHubActive != wasHubActive) {
            CommandScheduler.getInstance().schedule(
                Commands.startEnd(
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0.7);
                    operator.getHID().setRumble(RumbleType.kBothRumble, 0.7);
                },
                () -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                    operator.getHID().setRumble(RumbleType.kBothRumble, 0);
                }
            ).withTimeout(0.2));
        }
        wasHubActive = isHubActive;

        if (!isHubActive) {
            setSolidColor(Color.kRed);
            return;
        }

        Pose2d robotPose = drivetrain.getState().Pose;

        boolean isAligned = isAligned(robotPose);
        m_canShootPub.set(isAligned);

        if (isAligned) {
            setSolidColor(Color.kGreen);
            return;
        }

        if (isSwerveCommandRunning() && isInAllianceZone(robotPose)) {
            setSolidColor(Color.kYellow);
            return;
        }

        setSolidColor(Color.kOrange);
    }

    private boolean isAligned(Pose2d robotPose) {
        Translation2d target = GetTargetLocation.getTargetLocation(robotPose, drivetrain.getState().Speeds);
        if (target == null) return false;

        Pose2d shooterPose = robotPose.transformBy(SwerveConstants.robotToShooter);
        Rotation2d angleToTarget = target.minus(shooterPose.getTranslation()).getAngle();
        Rotation2d requiredRobotRotation = angleToTarget.minus(SwerveConstants.robotToShooter.getRotation());
        
        double error = Math.abs(robotPose.getRotation().minus(requiredRobotRotation).getDegrees());
        return error < LEDConstants.kMaxHeadingError;
    }

    private boolean isSwerveCommandRunning() {
        Command currentCommand = drivetrain.getCurrentCommand();
        return currentCommand != null && currentCommand != drivetrain.getDefaultCommand();
    }

    private boolean isInAllianceZone(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        if (alliance.get() == Alliance.Blue) {
            return robotPose.getX() < SwerveConstants.allianceZoneWidth;
        } else {
            return robotPose.getX() > (SwerveConstants.fieldWidth - SwerveConstants.allianceZoneWidth);
        }
    }

    public void setSolidColor(Color color, double brightness){
        candle.setControl(new SolidColor(0, LEDConstants.endIndex).withColor(new RGBWColor(color).scaleBrightness(brightness)));
    }

    public void setSolidColor(Color color){
        this.setSolidColor(color, 1);
    }
    
    public void clearColor(){
        candle.setControl(new SolidColor(0, LEDConstants.endIndex).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));
    }
    
    public void startFireAnimation(){
        FireAnimation FIRE = new FireAnimation(0, LEDConstants.endIndex).withBrightness(1).withCooling(0.3);
        candle.setControl(FIRE);   
    }

}