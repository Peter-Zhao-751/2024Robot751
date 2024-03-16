package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// POV import
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.lowLevelCommands.IntakeCommand;
import frc.robot.commands.lowLevelCommands.ShootCommand;
import frc.robot.commands.lowLevelCommands.IntakeCommand.IntakeSwivelMode;
import frc.robot.subsystems.*;
import frc.robot.utility.Barn2PathInterpreter;
import frc.robot.utility.PS5Controller;
import java.util.Optional;

import java.io.File;

public class RobotContainer {

    /* Commands */
    
    //private final Command Shooter;
    //private final Command Intake;

    /* Controllers */
    private final PS5Controller driver = new PS5Controller(1);
    //private final PS5Controller operator = new PS5Controller(1);

    /* Subsystems */
    //private final CANdle s_CANdle = new CANdle();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    //private final ClimberSubsystem s_Climber = new ClimberSubsystem();
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final TransferSubsystem s_Transfer = new TransferSubsystem();
    private final PowerSubsystem s_PDH = new PowerSubsystem();

    private final Barn2PathInterpreter u_Barn2PathInterpreter = new Barn2PathInterpreter(s_Intake, s_Transfer, s_Shooter, s_Swerve);

    /* values */
    private boolean precise = false;


    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Shooter = new Shoot(s_Shooter, s_Transfer, 20, false);
        //Intake = new Intake(s_Intake, s_Transfer);
        //new IntakeCommand(s_Intake, s_Transfer, IntakeSwivelMode.Extend, false);

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
        CANdleController.setCandle(new CANdleSubsystem());
    }
    
    private void configureButtonBindings() {
        // Util Commands (Circle, Triangle, X)
        driver.circleButton.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        driver.triangleButton.onTrue(new InstantCommand(s_Swerve::resetModulesToAbsolute));
        Optional<Alliance> alliance = DriverStation.getAlliance();

        Pose2d currentPose = s_Swerve.getSwerveOdometryPose2d();
        
        driver.crossButton.onTrue(new InstantCommand(() -> {
            if (alliance.isPresent() && alliance.get() == Alliance.Blue) new MoveCommand(s_Swerve, new Pose2d(currentPose.getX() + 300, currentPose.getY(), currentPose.getRotation()));
            else new MoveCommand(s_Swerve, new Pose2d(currentPose.getX() - 300, currentPose.getY(), currentPose.getRotation()));
        }));

        //driver.crossButton.onTrue(new InstantCommand(s_Swerve::crossModules));
        driver.squareButton.whileTrue(new IntakeCommand(s_Intake, s_Transfer, IntakeSwivelMode.Amp, precise));

        // // Precise Control (Left Bumper)
        driver.leftBumper.whileTrue(new InstantCommand(() -> precise = true));
        driver.leftBumper.onFalse(new InstantCommand(() -> precise = false));

        // Shooter & Intake (Left & Right Triggers)
        driver.rightTrigger.whileTrue(new ShootCommand(s_Shooter, s_Transfer, 110, false));
        driver.leftTrigger.whileTrue(new IntakeCommand(s_Intake, s_Transfer, IntakeSwivelMode.Extend, false));

        // Aimbot (Right Bumper)
        //driver.rightBumper.whileTrue(new AimbotCommand(s_Swerve));

        // FOR SYSID PID TUNING
        // driver.rightBumper.whileTrue(s_Intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driver.rightTrigger.whileTrue(s_Intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driver.leftBumper.whileTrue(s_Intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driver.leftTrigger.whileTrue(s_Intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Climber (D-Pad)
        //driver.dPad.whileUp(); // go up
        //driver.dPad.whileDown(); // go down
        //driver.dPad.whileLeft(); // automatic climb

        // driver.rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(60.0)));
        // driver.rightBumper.whileFalse(new InstantCommand(() -> s_Transfer.setShooterTransfer(0)));
        // driver.leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(10.0)));

        //driver.rightBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(40)));
        //driver.squareButton.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.IntakePositions.INTAKE)));
        //driver.leftBumper.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.IntakePositions.RETRACTED)));

    //    driver.rightBumper.onTrue(new InstantCommand(() -> {
    //        s_Shooter.setSpeed(20);
    //        s_Intake.setIntakeSpeed(20);
    //        s_Transfer.setIntakeTransfer(30);
    //        s_Transfer.setShooterTransfer(20);
    //    }));

    //    driver.rightBumper.onFalse(new InstantCommand(() ->  {
    //        s_Shooter.setSpeed(0);
    //        s_Intake.stopAll();
    //        s_Transfer.stop();
    //    }));

//        driver.rightTrigger.whileTrue(new ShootCommand(s_Shooter, s_Transfer, s_Swerve));
//        driver.leftTrigger.whileTrue(new IntakeCommand(s_Intake, s_Transfer));
//        driver.leftBumper.whileTrue(new TransferCommand(s_Transfer));

        //driver.rightTrigger.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //driver.rightBumper.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    }

    public Command getAutonomousCommand() {
        return new AutonCommand(s_Swerve);
    }

    public Command getAutonomousCommand(File path){
        if (path != null){
            try { return new AutonCommand(s_Swerve, u_Barn2PathInterpreter.getAutonCommands(path));}
            catch (Exception e) { System.err.println("Error: " + e); System.err.println("something stupid happened, probably petek's fault");}
        }
        System.err.println("No path file found");
        return new AutonCommand(s_Swerve);
    }
}