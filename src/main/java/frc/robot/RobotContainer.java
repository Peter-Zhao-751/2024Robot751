package frc.robot;

// POV import
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.lowLevelCommands.IntakeCommand;
import frc.robot.commands.lowLevelCommands.ShootCommand;
import frc.robot.commands.lowLevelCommands.IntakeCommand.IntakeSwivelMode;
//import frc.robot.commands.actuallyGoodCommands.IntakeCommand;
//import frc.robot.commands.actuallyGoodCommands.ShootCommand;
//import frc.robot.commands.actuallyGoodCommands.TransferCommand;
import frc.robot.subsystems.*;
import frc.robot.utility.JsonParser;
import frc.robot.utility.PS5Controller;

import java.io.File;

public class RobotContainer {

    /* Commands */
    
    //private final Command Shooter;
    //private final Command Intake;

    /* Controllers */
    private final PS5Controller driver = new PS5Controller(0);
    private final PS5Controller operator = new PS5Controller(1);

    /* Subsystems */
    //private final CANdle s_CANdle = new CANdle();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    //private final ClimberSubsystem s_Climber = new ClimberSubsystem();
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final TransferSubsystem s_Transfer = new TransferSubsystem();
    private final PowerSubsystem s_PDH = new PowerSubsystem();

    private final JsonParser s_jsonParser = new JsonParser(s_Intake, s_Transfer, s_Shooter, s_Swerve);

    /* values */
    private boolean precise = false;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Shooter = new Shoot(s_Shooter, s_Transfer, 20, false);
        //Intake = new Intake(s_Intake, s_Transfer);

        s_Swerve.setDefaultCommand(
            new TeleopCommand(
                s_Swerve, 
                () -> -driver.joystick.getRawAxis(PS5Controller.translationAxis),
                () -> -driver.joystick.getRawAxis(PS5Controller.strafeAxis),
                () -> -driver.joystick.getRawAxis(PS5Controller.rotationAxis),
                () -> false,
                () -> precise
            )
        );

        configureButtonBindings();
    }
    
    private void configureButtonBindings() {
        /* Util Commands */
        driver.circleButton.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        driver.triangleButton.whileTrue(new InstantCommand(s_Swerve::resetModulesToAbsolute));

        driver.leftBumper.whileTrue(new InstantCommand(() -> precise = true));
        driver.leftBumper.onFalse(new InstantCommand(() -> precise = false));

        driver.rightTrigger.whileTrue(new ShootCommand(s_Shooter, s_Transfer, 0, false));
        driver.leftTrigger.whileTrue(new IntakeCommand(s_Intake, s_Transfer, IntakeSwivelMode.Extend, false));

        // SHOOTER STUFF
        // 42-44 seems to work well
        // driver.rightTrigger.onTrue(new InstantCommand(() -> s_Shooter.setSpeed((double) TelemetryUpdater.getTelemetryValue("Shooter Speed"))));
        // driver.rightTrigger.onFalse(new InstantCommand(() -> s_Shooter.stop()));

        // driver.rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(60.0)));
        // driver.rightBumper.whileFalse(new InstantCommand(() -> s_Transfer.setShooterTransfer(0)));
        // driver.leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(10.0)));

        //driver.rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(40)));
        //driver.squareButton.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.IntakePositions.INTAKE)));
        //driver.leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.IntakePositions.RETRACTED)));

        // TODO: #8 Run logging for the swerve drive
        // LOGGING STUFF FOR DRIVETRAIN
        //s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        //s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        driver.rightBumper.onTrue(new InstantCommand(() -> {
            s_Shooter.setSpeed(20);
            s_Intake.setIntakeSpeed(20);
            s_Transfer.setIntakeTransfer(30);
            s_Transfer.setShooterTransfer(20);
        }));

        driver.rightBumper.onFalse(new InstantCommand(() ->  {
            s_Shooter.setSpeed(0);
            s_Intake.stopAll();
            s_Transfer.stop();
        }));

//        driver.rightTrigger.whileTrue(new ShootCommand(s_Shooter, s_Transfer, s_Swerve));
//        driver.leftTrigger.whileTrue(new IntakeCommand(s_Intake, s_Transfer));
//        driver.leftBumper.whileTrue(new TransferCommand(s_Transfer));

        //driver.rightTrigger.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //driver.rightBumper.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        
        /* Drivetrain Commands */
        driver.crossButton.toggleOnTrue(new InstantCommand(s_Swerve::crossModules));
    }

    public Command getAutonomousCommand() {
        return new AutonCommand(s_Swerve);
    }

    public Command getAutonomousCommand(File path){
        if (path != null){
            try { return new AutonCommand(s_Swerve, s_jsonParser.getAutonCommands(path));}
            catch (Exception e) { System.out.println("Error: " + e); System.out.println("something stupid happened, probably owen's fault");}
        }
        System.out.println("No path file found");
        return new AutonCommand(s_Swerve);
    }
}