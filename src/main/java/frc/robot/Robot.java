// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FuelSim;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private NetworkTableInstance table = NetworkTableInstance.getDefault();
    private StructPublisher<Pose3d> intakepublisher = table.getStructTopic("/intake/pose", Pose3d.struct).publish();

    private StructPublisher<Pose3d> hoodpublisher = table.getStructTopic("/hood/pose", Pose3d.struct).publish();

    private StructPublisher<Pose3d> publisherzero = table.getStructTopic("/intake/pose2", Pose3d.struct).publish();


    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }
    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        Pose3d intakepose = new Pose3d(0.28,0,0.275, new Rotation3d(0,Math.sin(Timer.getTimestamp())+1.0,0));

        Pose3d zero = new Pose3d();
        intakepublisher.set(intakepose);
        publisherzero.set(zero);

        Pose3d hoodpose = new Pose3d(0,0,0.58, new Rotation3d(0,Math.sin(Timer.getTimestamp())-2.5,0));
        hoodpublisher.set(hoodpose);

        SmartDashboard.putNumber("Score", FuelSim.Hub.BLUE_HUB.getScore());


    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();

    }
}