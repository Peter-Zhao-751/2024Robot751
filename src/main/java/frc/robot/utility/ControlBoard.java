package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.lowLevelCommands.*;
import frc.robot.subsystems.*;

public class ControlBoard {
    private static ControlBoard instance;

    private static final double shooterSpeedIncrement = 0.02;
    private static final double intakeAngleIncrement = 1;

    /* Controllers */
    private final PS5Controller driver;
    private final PS5Controller operator;

    private final SwerveSubsystem s_Swerve;
    // private final IntakeSubsystem s_Intake;
    // private final ShooterSubsystem s_Shooter;
    // private final ClimberSubsystem s_Climber;

    private Mode currentMode = Mode.Speaker;
    private boolean precise = false;
    private double shooterSpeed = Constants.Shooter.maxShooterSpeed;

    public enum Mode {
        Speaker,
        Amp,
        Climb
    }

    private ControlBoard() {
        driver = new PS5Controller(1);
        operator = new PS5Controller(0);

        s_Swerve = SwerveSubsystem.getInstance();
        // s_Intake = IntakeSubsystem.getInstance();
        // s_Shooter = ShooterSubsystem.getInstance();
        // s_Climber = ClimberSubsystem.getInstance();

        s_Swerve.setDefaultCommand(
                new TeleopCommand(
                        () -> -driver.joystick.getRawAxis(PS5Controller.translationAxis),
                        () -> -driver.joystick.getRawAxis(PS5Controller.strafeAxis),
                        () -> -driver.joystick.getRawAxis(PS5Controller.rotationAxis),
                        () -> false,
                        () -> precise
                )
        );

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        driver.leftTrigger.whileTrue(new IntakeCommand());
        driver.leftBumper.whileTrue(new ExportCommand());
        driver.rightBumper.toggleOnTrue(new SpinShooterCommand());
        driver.rightTrigger.whileTrue(new ShootCommand());

        driver.dPad.up.onTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        driver.dPad.right.onTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        //driver.dPad.left.whileTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.kRetractedAngle)));
        driver.dPad.down.whileTrue(new TransferCommand());

        driver.triangleButton.whileTrue(new RunCommand(s_Swerve::resetModulesToAbsolute));
//        driver.squareButton.whileTrue(new InstantCommand(/*TODO Reset Odometry*/));
        driver.circleButton.whileTrue(new InstantCommand(s_Swerve::zeroHeading));
//        driver.crossButton.whileTrue(new InstantCommand(/*TODO Disable LEDs*/));

        driver.leftTriggerButton.onTrue(new InstantCommand(this::togglePrecise));
        driver.rightTriggerButton.whileTrue(new RunCommand(s_Swerve::crossWheels));
    }

    private void configureOperatorBindings() {
        // operator.leftTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        // operator.leftTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

        // operator.leftBumper.and(this::notClimberMode).whileTrue(new InstantCommand(/* TODO FLASH LEDs FOR HUMAN PLAYER*/));
        // operator.leftBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(Constants.Climber.climberSpeed), s_Climber));

        // operator.rightTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        // operator.rightTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

        // operator.rightBumper.and(this::notClimberMode).whileTrue(new TransferCommand());
        // operator.rightBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(Constants.Climber.climberSpeed), s_Climber));

        // operator.dPad.up.whileTrue(new RunCommand(this::increaseShooterPower, s_Shooter));
        // operator.dPad.down.whileTrue(new RunCommand(this::decreaseShooterPower, s_Shooter));
        // operator.dPad.left.whileTrue(new RunCommand(this::retractIntake, s_Intake));
        // operator.dPad.right.whileTrue(new RunCommand(this::extendIntake, s_Intake));

        // operator.triangleButton.toggleOnTrue(new StartEndCommand(() -> currentMode = Mode.Climb, () -> currentMode = Mode.Speaker));

        // operator.squareButton // TODO TBD
        operator.circleButton.whileTrue(new InstantCommand(this::togglePrecise));
        operator.crossButton.whileTrue(new InstantCommand(s_Swerve::crossWheels, s_Swerve));
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putString("Mode", currentMode.toString());
    }

    public double shooterSpeed() {
        return shooterSpeed;
    }

    private void retractIntake() {
        //s_Intake.setSwivelPosition(s_Intake.getSwivelPosition() - intakeAngleIncrement);
    }

    private void extendIntake() {
        //s_Intake.setSwivelPosition(s_Intake.getSwivelPosition() + intakeAngleIncrement);
    }

    private void togglePrecise() {
        precise = !precise;
    }

    private void increaseShooterPower() {
        shooterSpeed += shooterSpeedIncrement;
    }

    private void decreaseShooterPower() {
        shooterSpeed -= shooterSpeedIncrement;
    }

    public Mode getMode() {
        return currentMode;
    }

    private boolean notClimberMode() {
        return currentMode != Mode.Climb;
    }

    private boolean climberMode() {
        return currentMode == Mode.Climb;
    }
}
