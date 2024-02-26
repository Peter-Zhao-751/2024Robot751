// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CurrentManager;
import java.io.File;
import java.nio.file.Path;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.Base64;
import edu.wpi.first.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import frc.robot.subsystems.JsonParser;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private final SendableChooser<File> autonSelector = new SendableChooser<>();
  private GenericEntry updatePreferencesButton;
  private GenericEntry resetPreferencesButton;
  private File selectedAuton = null;
  private String base64Image = null;
  private CvSource imageSource;

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
    updatePreferencesBasedOnConstants(Constants.class, false);

    // robot container
    m_robotContainer = new RobotContainer();
    SignalLogger.setPath("/media/sda1/");
    initializeUI();
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
    updateTelemetry();
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
    File selectedAuton = autonSelector.getSelected();
    SmartDashboard.putString("Current Action", "Autonomous: " + selectedAuton.getName());
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
    currentMode = RobotModes.Test;
    SignalLogger.start();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  // updating ui methods
  private void updatePathPreview(){
    File currentSelection = autonSelector.getSelected();
    if(!(currentSelection == null) && !currentSelection.equals(selectedAuton)){
      selectedAuton = currentSelection;
      if (selectedAuton != null) {
        base64Image = JsonParser.getAutonPreview(selectedAuton);
        System.out.println("\n\nupdated auton path\n\n");
        byte [] base64ImageByte = Base64.getDecoder().decode(base64Image);
        Mat image = Imgcodecs.imdecode(new MatOfByte(base64ImageByte), Imgcodecs.IMREAD_UNCHANGED);
        imageSource.putFrame(image);
      }
    }
  }

  /**
   * This function will go through the provided class (the Constants class) and update each
   * preference based on the value of the constant. If the preference does not exist, it will be
   * created. If it does exist, the Constant will be updated to match the preference.
   */
  private void updatePreferencesBasedOnConstants(Class<?> clazz, boolean reset) {
      Field[] fields = clazz.getDeclaredFields();
      for (Field field : fields) {
          if (Modifier.isPublic(field.getModifiers()) && Modifier.isStatic(field.getModifiers())) {
              if (reset) resetPreferenceToConstant(field);
              else updateOrRetrievePreference(field);
          }
      }

      // Recursively handle inner classes
      Class<?>[] innerClasses = clazz.getDeclaredClasses();
      for (Class<?> innerClass : innerClasses) {
          updatePreferencesBasedOnConstants(innerClass, reset);
      }
  }

  /**
   * This function will update the preference based on the value of the field, or retrieve the
   * preference and update the field's value based on the preference.
   */
  private void updateOrRetrievePreference(Field field) {
      try {
          // Ensure the field is accessible
          field.setAccessible(true);
          // Check if a Preference exists for this field
          if (!Preferences.containsKey(field.getName())) {
              switch (field.getType().getName()) {
                case "int":
                    Preferences.setInt(field.getName(), field.getInt(null));
                    break;
                case "double":
                    Preferences.setDouble(field.getName(), field.getDouble(null));
                    break;
                case "boolean":
                    Preferences.setBoolean(field.getName(), field.getBoolean(null));
                    break;
                case "java.lang.String":
                    Preferences.setString(field.getName(), (String) field.get(null));
                    break;
              }
          } else {
              // Preference exists, update field's value from Preference
              switch (field.getType().getName()) {
                case "int":
                    field.setInt(null, Preferences.getInt(field.getName(), field.getInt(null)));
                    break;
                case "double":
                    field.setDouble(null, Preferences.getDouble(field.getName(), field.getDouble(null)));
                    break;
                case "boolean":
                    field.setBoolean(null, Preferences.getBoolean(field.getName(), field.getBoolean(null)));
                    break;
                case "java.lang.String":
                    field.set(null, Preferences.getString(field.getName(), (String) field.get(null)));
                    break;
              }
          }
      } catch (IllegalArgumentException | IllegalAccessException e) {
          e.printStackTrace();
      }
  }

  /**
   * This function will reset the preference based on the value of the field, or retrieve the
   * preference and reset the field's value based on the preference.
   */
  private void resetPreferenceToConstant(Field field) {
      try {
          // Ensure the field is accessible
          field.setAccessible(true);
          // Check if a Preference exists for this field
          if (Preferences.containsKey(field.getName())) {
              // Preference exists, update preferences's value from field
              switch (field.getType().getName()) {
                case "int":
                    Preferences.setInt(field.getName(), field.getInt(null));
                    break;
                case "double":
                    Preferences.setDouble(field.getName(), field.getDouble(null));
                    break;
                case "boolean":
                    Preferences.setBoolean(field.getName(), field.getBoolean(null));
                    break;
                case "java.lang.String":
                    Preferences.setString(field.getName(), (String) field.get(null));
                    break;
              }
          }
      } catch (IllegalArgumentException | IllegalAccessException e) {
          e.printStackTrace();
      }
  }

  private void updateTelemetry(){
    SmartDashboard.putBoolean("Current Manager Over Nominal", CurrentManager.isOverNominal());
    SmartDashboard.putBoolean("Current Manager Over Peak", CurrentManager.isOverMax());
    
    if (updatePreferencesButton.getBoolean(false)) {
        updatePreferencesBasedOnConstants(Constants.class, false);
        updatePreferencesButton.setBoolean(false); // Reset the toggle button
    }
    if (resetPreferencesButton.getBoolean(false)) {
        updatePreferencesBasedOnConstants(Constants.class, true);
        resetPreferencesButton.setBoolean(false); // Reset the toggle button
    }
    updatePathPreview();
  }

  private void initializeUI(){
    // initializing limelight
    HttpCamera limelightStream = new HttpCamera("LimelightStream", "http://10.7.51.11:5800", HttpCameraKind.kMJPGStreamer);
    CameraServer.addCamera(limelightStream);
    SmartDashboard.putNumber("Shooter Speed", 0);

    // initializing webcam
    //UsbCamera webcam = new UsbCamera("WebcameStream", 0);
    //webcam.setResolution(640, 480);
    //CameraServer.addCamera(webcam);

    CameraServer.startAutomaticCapture(limelightStream);

    imageSource = CameraServer.putVideo("Path Preview", 827, 401);
    CameraServer.startAutomaticCapture(imageSource);

    //sets a bunch of UI stuff
    Path deployDirectory = Filesystem.getDeployDirectory().toPath();
    Path barn2PathDirectory = deployDirectory.resolve("barn2path");

    File barn2PathDir = barn2PathDirectory.toFile();
    File[] filesList = barn2PathDir.listFiles();

    autonSelector.setDefaultOption("Simple Auton", null);
    for (File file : filesList){
      autonSelector.addOption(file.getName(), file);
    }
    
    Shuffleboard.getTab("Auton Selector")
      .add("Select a Path:", autonSelector)
      .withWidget("Combo Box Chooser");

    SmartDashboard.putString("Current Mode", currentMode.toString());

    updatePreferencesButton = Shuffleboard.getTab("Preferences")
    .add("Update Preferences", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .getEntry();

    resetPreferencesButton = Shuffleboard.getTab("Preferences")
    .add("Reset Preferences", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .getEntry();
  }
}
