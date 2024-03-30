package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.commands.movementCommands.TeleopCommand;
import frc.robot.commands.nonMovementCommands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ControlBoard {
    private static ControlBoard instance;

    private static final double shooterSpeedIncrement = 0.02;
    private static final double intakeAngleIncrement = 1;

    /* Controllers */
    private final PS5Controller driver;
    private final PS5Controller operator;

    private final SwerveSubsystem s_Swerve;
    private final IntakeSubsystem s_Intake;
    private final ShooterSubsystem s_Shooter;
    // private final ClimberSubsystem s_Climber;

    private Mode currentMode = Mode.Speaker;
    private boolean precise = false;
    private boolean fieldCentric = true;
    private boolean zeroing = false;
    private double shooterSpeed = Constants.Shooter.maxShooterSpeed;

    public enum Mode {
        Speaker,
        Amp,
        Climb
    }

    private ControlBoard() {
        driver = new PS5Controller(0);
        operator = new PS5Controller(1);

        s_Swerve = SwerveSubsystem.getInstance();
        s_Intake = IntakeSubsystem.getInstance();
        s_Shooter = ShooterSubsystem.getInstance();
        // s_Climber = ClimberSubsystem.getInstance();

        s_Swerve.setDefaultCommand(
                new TeleopCommand(
                        driver.leftVerticalJoystick,
                        driver.leftHorizontalJoystick,
                        driver.rightHorizontalJoystick,
                        driver.rightJoystickButton,
                        driver.leftJoystickButton,
                        driver.triangleButton.or(operator.crossButton) // Zeroing wheels or cross wheels
                )
        );

        configureDriverBindings();
        //    configureRoutines();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        driver.leftTrigger.whileTrue(new IntakeCommand());
        driver.leftBumper.whileTrue(new ExportCommand());
        driver.rightBumper.toggleOnTrue(new SpinShooterCommand());
        driver.rightTrigger.whileTrue(new ShootCommand());

        driver.dUp.onTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        driver.dRight.onTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        driver.dLeft.whileTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.kRetractedAngle)));
        driver.dDown.whileTrue(new TransferCommand());

        driver.triangleButton.whileTrue(new InstantCommand(s_Swerve::resetModulesToAbsolute));
        driver.squareButton.whileTrue(new InstantCommand(StateEstimator.getInstance()::resetPose));
        driver.circleButton.whileTrue(new InstantCommand(s_Swerve::zeroHeading));
        driver.crossButton.whileTrue(new InstantCommand(LimelightSubsystem.getInstance()::toggleLeds)); // TODO: add on CANdle disabling
    }

    private void configureRoutines() {

    }

    private void configureOperatorBindings() {
        operator.leftTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        // operator.leftTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

        operator.leftBumper.and(this::notClimberMode).whileTrue(new StartEndCommand(
                () -> LimelightSubsystem.getInstance().setLEDMode(LimelightSubsystem.LEDMode.BLINK),
                () -> LimelightSubsystem.getInstance().setLEDMode(LimelightSubsystem.LEDMode.OFF)
        ));
        // operator.leftBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(Constants.Climber.climberSpeed), s_Climber));

        operator.rightTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        // operator.rightTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

         operator.rightBumper.and(this::notClimberMode).whileTrue(new TransferCommand());
        // operator.rightBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(Constants.Climber.climberSpeed), s_Climber));

         operator.dUp.whileTrue(new RunCommand(this::increaseShooterPower, s_Shooter));
         operator.dDown.whileTrue(new RunCommand(this::decreaseShooterPower, s_Shooter));
         operator.dLeft.whileTrue(new RunCommand(this::retractIntake, s_Intake));
         operator.dRight.whileTrue(new RunCommand(this::extendIntake, s_Intake));

        operator.triangleButton.toggleOnTrue(new StartEndCommand(() -> currentMode = Mode.Climb, () -> currentMode = Mode.Speaker));
        operator.squareButton.onTrue(new InstantCommand(() -> IntakeSubsystem.getInstance().setSwivelPosition(Constants.Intake.kRetractedAngle)));
        operator.circleButton.whileTrue(new InstantCommand(this::togglePrecise));
        operator.crossButton.whileTrue(new InstantCommand(s_Swerve::crossWheels, s_Swerve));
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("Target Shooter Speed", shooterSpeed);
        SmartDashboard.putString("Mode", currentMode.toString());
    }

    private void togglePrecise() {
        precise = !precise;
    }

    public double shooterSpeed() {
        return shooterSpeed;
    }

    private void retractIntake() {
        s_Intake.setSwivelPosition(s_Intake.getSwivelPosition() - intakeAngleIncrement);
    }

    private void extendIntake() {
        s_Intake.setSwivelPosition(s_Intake.getSwivelPosition() + intakeAngleIncrement);
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

    private boolean climberMode() {
        return currentMode == Mode.Climb;
    }

    private boolean notClimberMode() {
        return currentMode != Mode.Climb;
    }
}
