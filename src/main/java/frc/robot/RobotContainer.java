package frc.robot;

import javax.swing.JList.DropLocation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// POV import
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.lowLevelCommands.Intake;
import frc.robot.commands.lowLevelCommands.Shoot;
import frc.robot.commands.lowLevelCommands.Transfer;
import frc.robot.subsystems.*;
import frc.robot.utility.JsonParser;
import frc.robot.utility.TelemetryUpdater;

import java.io.File;

public class RobotContainer {
    /*  
     * precise control: left bumper
     * aimbot: right bumper
     * shoot: right trigger
     * intake: left trigger
     * 
     * zero pigeon (for field based control and stuff): triangle
     * zero modules (should be automatic, but incase somehting goes wrong in the game): circle
     * cross wheels (maybe auto do this when shooting?): square
     * 
     * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PS5Controller.Button.html
     */

    private boolean climberMode = false;

    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Joysticks */

    private final int translationAxis = PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;

    /* Triggers */
    
    private final JoystickButton leftTrigger = new JoystickButton(driver, PS5Controller.Button.kL2.value); // INTAKE
    private final JoystickButton rightTrigger = new JoystickButton(driver, PS5Controller.Button.kR2.value); // SHOOT

    /* Trigger Buttons */

    private final JoystickButton leftTriggerButton = new JoystickButton(driver, PS5Controller.Button.kL3.value); // unassigned
    private final JoystickButton rightTriggerButton = new JoystickButton(driver, PS5Controller.Button.kR3.value); // unassigned
    
    /* Bumpers */
    
    private final JoystickButton leftBumper = new JoystickButton(driver, PS5Controller.Button.kL1.value); // AIM BOT
    private final JoystickButton rightBumper = new JoystickButton(driver, PS5Controller.Button.kR1.value); // PRECISE CONTROL
    
    /* Buttons */

    private final JoystickButton triangleButton = new JoystickButton(driver, PS5Controller.Button.kTriangle.value); 
    private final JoystickButton circleButton = new JoystickButton(driver, PS5Controller.Button.kCircle.value);
    private final JoystickButton squareButton = new JoystickButton(driver, PS5Controller.Button.kSquare.value);
    private final JoystickButton crossButton = new JoystickButton(driver, PS5Controller.Button.kCross.value);

    /* Other Buttons */

    private final JoystickButton optionsButton = new JoystickButton(driver, PS5Controller.Button.kOptions.value); // ZERO PIGEON
    private final JoystickButton playstationButton = new JoystickButton(driver, PS5Controller.Button.kPS.value); // ZERO MODULES

    /* Commands */
    
    private final Command Shooter;
    //private final Command Intake;

    /* Subsystems */
    //private final CANdle s_CANdle = new CANdle();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    //private final ClimberSubsystem s_Climber = new ClimberSubsystem();
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final TransferSubsystem s_Transfer = new TransferSubsystem();

    /* values */
    private boolean precise = false;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Shooter = new Shoot(s_Shooter, s_Transfer, 200, false);
        //Intake = new Intake(s_Intake, s_Transfer);

        JsonParser.JsonParser(null, s_Transfer, s_Shooter, s_Swerve);

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
        /* Util Commands */
        circleButton.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        triangleButton.whileTrue(new InstantCommand(() -> {
            s_Swerve.resetModulesToAbsolute();
        }));

        //leftBumper.whileTrue(new InstantCommand(() -> precise = true));
        //leftBumper.onFalse(new InstantCommand(() -> precise = false));


        // SHOOTER STUFF
        // 42-44 seems to work well
        // rightTrigger.onTrue(new InstantCommand(() -> s_Shooter.setSpeed((double) TelemetryUpdater.getTelemetryValue("Shooter Speed"))));
        // rightTrigger.onFalse(new InstantCommand(() -> s_Shooter.stop()));

        // rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(60.0)));
        // //rightBumper.whileFalse(new InstantCommand(() -> s_Transfer.setShooterTransfer(0)));
        // leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(10.0)));

        // LOGGING STUFF FOR DRIVETRAIN
        // TODO: #8 Run logging for the swerve drive
        rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(40)));//s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        leftBumper.onFalse(new InstantCommand(() -> s_Intake.setSwivelPosition(60)));//s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        
        rightBumper.onTrue(new InstantCommand(() -> s_Intake.setIntakeSpeed(20)));
        //rightTrigger.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //rightBumper.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        
        /* Drivetrain Commands */
        crossButton.toggleOnTrue(new InstantCommand(() -> s_Swerve.crossModules()));
    }

    public Command getAutonomousCommand() {
        return new Auton(s_Swerve);
    }

    public Command getAutonomousCommand(File path){
        if (path != null){
            try { return new Auton(s_Swerve, JsonParser.getAutonCommands(path));}     
            catch (Exception e) { System.out.println("Error: " + e);}
        }
        return new Auton(s_Swerve);
    }
}