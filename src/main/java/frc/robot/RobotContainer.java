package frc.robot;

import javax.swing.JList.DropLocation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller;
// POV import
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    /* PS5 Versions are commented below the Xbox implementation. */
    
    /*private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int armAxis = XboxController.Axis.kRightY.value;*/
    
    private final int translationAxis = PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;
    
    

    /* Bumper Buttons */
    
    private final JoystickButton intakeButton = new JoystickButton(driver, PS5Controller.Button.kL2.value);
    private final JoystickButton shootButton = new JoystickButton(driver, PS5Controller.Button.kR2.value);
    
    /* Driver Buttons */
    /*private final JoystickButton preciseControl = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aimBot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);*/
    
    private final JoystickButton preciseControl = new JoystickButton(driver, PS5Controller.Button.kL1.value);
    private final JoystickButton aimBot = new JoystickButton(driver, PS5Controller.Button.kR1.value);
    

    /*private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroModules = new JoystickButton(driver, XboxController.Button.kX.value);*/
    
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS5Controller.Button.kTriangle.value);
    private final JoystickButton zeroModules = new JoystickButton(driver, PS5Controller.Button.kCircle.value);
    

    /* Climb Buttons */
    /*
    private final POVButton climbUp = new POVButton(driver, 0);
    private final POVButton climbDown = new POVButton(driver, 180);
    */
    
    /* Commands */
    private final Command Shooter;
    private final Command Intake;

    /* Subsystems */
    //private final CANdle s_CANdle = new CANdle();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final SwerveDrive s_Swerve = new SwerveDrive();
    private final CANdleSubsystem s_CANdle = new CANdleSubsystem();


    private final JsonParser jsonParser;

    /* values */
    private boolean precise = false;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Shooter = new Shooter(s_Shooter);
        Intake = new Intake(s_Intake);

        jsonParser = new JsonParser(s_Intake, s_Shooter, s_Swerve);

        s_Swerve.setDefaultCommand(
            new Teleop(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> precise
            )
        );

        configureButtonBindings();
    }
    
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        zeroModules.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        preciseControl.whileTrue(new InstantCommand(() -> precise = true));
        preciseControl.onFalse(new InstantCommand(() -> precise = false));

        intakeButton.whileTrue(new Intake(s_Intake));
        shootButton.whileTrue(new Shooter(s_Shooter));
        aimBot.whileTrue(new AimBot(s_Swerve));
    }

    public Command getAutonomousCommand() {
        return new Auton(s_Swerve);
    }

    public Command getAutonomousCommand(File path){
        if (path != null){
            try { return new Auton(s_Swerve, jsonParser.getAutonCommands(path));}     
            catch (Exception e) { System.out.println("Error: " + e);}
        }
        return new Auton(s_Swerve);
    }

    public String getAutonomousPreview(File path){
        if (path != null){
            try { return jsonParser.getAutonPreview(path);}     
            catch (Exception e) { System.out.println("Error: " + e);}
        }
        return "No Path";
    }
}
