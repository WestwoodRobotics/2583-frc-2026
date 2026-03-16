package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {
    private final TalonFX m_floorMotor1 = new TalonFX(TransferConstants.kFloorId1, new CANBus(TransferConstants.kCANBus));
    private final TalonFX m_transferMotor1 = new TalonFX(TransferConstants.kTransferId1, new CANBus(TransferConstants.kCANBus));
    private final TalonFX m_transferMotor2 = new TalonFX(TransferConstants.kTransferId2, new CANBus(TransferConstants.kCANBus));

    private final VelocityTorqueCurrentFOC m_floorRequest = new VelocityTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC m_transferRequest = new VelocityTorqueCurrentFOC(0);

    private final VoltageOut m_voltReq = new VoltageOut(0.0);

    private final Follower m_transferInvertedFollower = new Follower(TransferConstants.kTransferId1, MotorAlignmentValue.Opposed);

    private double jamTimeStart = 0.0;
    private double reverseTimeStart = 0.0;

    private Timer  debounceTimer = new Timer();
    private Timer  reverseTimer = new Timer();


    private final SysIdRoutine m_floorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("floor_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_floorMotor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_transferSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> SignalLogger.writeString("transfer_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                m_transferMotor1.setControl(m_voltReq.withOutput(volts.in(Volts)));
                m_transferMotor2.setControl(m_voltReq.withOutput(-volts.in(Volts)));
            }, 
            null, 
            this)
    );

    private final SysIdRoutine m_routineToApply = m_transferSysIdRoutine;

    public Transfer() {
        m_floorMotor1.getConfigurator().apply(TransferConstants.getFloorMotorConfigs());
        m_transferMotor1.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        m_transferMotor2.getConfigurator().apply(TransferConstants.getTransferMotorConfigs());
        SignalLogger.start();
    }

    public void runMotors(double floorVel, double transferVel) {
        m_floorMotor1.setControl(m_floorRequest.withVelocity(floorVel));
        m_transferMotor1.setControl(m_transferRequest.withVelocity(transferVel));
        m_transferMotor2.setControl(m_transferInvertedFollower);
    }

    @Override
    public void periodic(){
        SignalLogger.writeDouble("Transfer floor current", this.m_floorMotor1.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Transfer 1 current", this.m_transferMotor1.getSupplyCurrent().getValueAsDouble());
        SignalLogger.writeDouble("Transfer 2 current", this.m_transferMotor2.getSupplyCurrent().getValueAsDouble());

    }

    public Command defaultCommand() {
        // Default: Floor spins at default speed, Transfer is stopped (default speed is 0.0 in constants)
        return Commands.run(() -> runMotors(TransferConstants.kFloorDefaultVel, TransferConstants.kTransferDefaultVel), this);
    }

    public Command shootCommand() {
        // Right Trigger: Both motors spin at their respective shooting velocities
        return Commands.run(() -> runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel), this);

    }

    public Command jamDetection() {

        return Commands.run( 
            ()-> {
                double transferMotor1Vel = Math.abs(this.m_transferMotor1.getVelocity().getValueAsDouble());
                double transferMotor2Vel = Math.abs(this.m_transferMotor2.getVelocity().getValueAsDouble());
                boolean jammed = (TransferConstants.kTransferShootVel - transferMotor1Vel > TransferConstants.kVelThreshold)
                || (TransferConstants.kTransferShootVel - transferMotor2Vel > TransferConstants.kVelThreshold);
                //not jammed, run normally
                if(!jammed){
                    debounceTimer.reset();
                    reverseTimer.reset();
                    runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel);
                }
                //jammed, start the debounce timer
                else {
                    debounceTimer.start();

                    if(debounceTimer.get() >= TransferConstants.kJamDebounceTime){
                        //reverse transfer if we have met the jam time requirements
                        runMotors(TransferConstants.kFloorShootVel,-TransferConstants.kTransferShootVel);
                        reverseTimer.start();
                        
                        if(reverseTimer.get() >= TransferConstants.kJamReverseTime){
                            //go back to normal if we have reversed for long enough
                            debounceTimer.reset();
                            reverseTimer.reset();
                            reverseTimer.stop();
                            debounceTimer.stop();
                            runMotors(TransferConstants.kFloorShootVel, TransferConstants.kTransferShootVel);
                        }
                    }
                }
            }
        
            );
        }

        
    

    public Command reverseCommand() {
        return Commands.run(() -> runMotors(-TransferConstants.kFloorShootVel, -TransferConstants.kTransferShootVel), this)
            .beforeStarting(() -> {
                TorqueCurrentConfigs config = new TorqueCurrentConfigs();
                config.PeakReverseTorqueCurrent = -800.0;
                config.PeakForwardTorqueCurrent = 0.0;
                m_transferMotor1.getConfigurator().apply(config);
                m_transferMotor2.getConfigurator().apply(config);
            })
            .finallyDo((interrupted) -> {
                TorqueCurrentConfigs config = new TorqueCurrentConfigs();
                config.PeakReverseTorqueCurrent = 0.0;
                config.PeakForwardTorqueCurrent = 800.0;
                m_transferMotor1.getConfigurator().apply(config);
                m_transferMotor2.getConfigurator().apply(config);
            });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_routineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_routineToApply.dynamic(direction);
    }
}