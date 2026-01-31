package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Flywheel extends SubsystemBase {
    private TalonFX flywheelMotor;
    private TalonFXConfiguration talonFXConfigs;
    private MotionMagicVelocityVoltage velocityControl;
    

    public Flywheel(TalonFX flywheelMotor) {
        this.flywheelMotor = flywheelMotor;
        talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Slot0.kS = 0.25; 
        talonFXConfigs.Slot0.kV = 0.12; 
        talonFXConfigs.Slot0.kA = 0.01; 
        talonFXConfigs.Slot0.kP = 4.8; 
        talonFXConfigs.Slot0.kI = 0; 
        talonFXConfigs.Slot0.kD = 0.1; 
        flywheelMotor.getConfigurator().apply(talonFXConfigs);

        velocityControl = new MotionMagicVelocityVoltage(0);
        SmartDashboard.putBoolean("here", true);
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        flywheelMotor.setControl(new VoltageOut(2));


    }
}