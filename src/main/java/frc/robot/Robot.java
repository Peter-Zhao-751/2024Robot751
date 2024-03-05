// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.File;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.utility.UISubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private static enum RobotModes {
    Disabled,
    Autonomous,
    Teleop,
    Test
  }

  private static RobotModes currentMode = RobotModes.Disabled;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Robot Preferences

    // Go through every constant in Constants.java check if the Preference exists.
    // add it if it doesn't. if it does, overwrite it.
    UISubsystem.updatePreferencesBasedOnConstants(Constants.class, false);

    // robot container
    m_robotContainer = new RobotContainer();

    SignalLogger.setPath("/media/sda1/");
    UISubsystem.initializeUI(currentMode);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    // updating ui
    UISubsystem.updateTelemetry();
    UISubsystem.updatePathPreview();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    currentMode = RobotModes.Disabled;
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    currentMode = RobotModes.Autonomous;
    File selectedAuton = UISubsystem.selectedAuton();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(selectedAuton);
    SignalLogger.start();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    SignalLogger.start();

    currentMode = RobotModes.Teleop;
    SmartDashboard.putString("Current Action", "Standard teleop");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("CAN Bus Utilization", CANBus.getStatus(Constants.CANivoreID).BusUtilization * 100 + "%");
  }

  @Override
  public void testInit() {
    SignalLogger.start();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
