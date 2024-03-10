package frc.robot;

// POV import
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
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
        //rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(40)));//s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driver.squareButton.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(5)));
        driver.leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(140)));//s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

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
    
        //rightTrigger.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //rightBumper.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        
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