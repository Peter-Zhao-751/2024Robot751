package frc.robot;

import javax.swing.JList.DropLocation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


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
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int armAxis = XboxController.Axis.kRightY.value;

    /* Bumper Buttons */
    private final int intakeAxis = XboxController.Axis.kRightTrigger.value;
    private final int shootAxis = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    private final JoystickButton preciseControl = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aimBot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroModules = new JoystickButton(driver, XboxController.Button.kX.value);
    
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        zeroModules.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        preciseControl.onTrue(new InstantCommand(() -> precise = !precise));

        new Trigger(() -> driver.getRawAxis(intakeAxis) > 0.5)
        .onTrue(new Shooter(s_Shooter));

        new Trigger(() -> driver.getRawAxis(shootAxis) > 0.5)
        .onTrue(new Intake(s_Intake));
        
        aimBot.toggleOnTrue(new AimBot(s_Swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new Auton(s_Swerve);
    }

    public Command getAutonomousCommand(String path){
        try {
            return new Auton(s_Swerve, jsonParser.getAutonCommands(path+".b2path"));
        } catch (Exception e) {
            System.out.println("Error: " + e);
            return new Auton(s_Swerve);
        }
    }
}
