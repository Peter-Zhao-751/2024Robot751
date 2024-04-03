package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.commands.movementCommands.*;
import frc.robot.commands.gamepieceCommands.*;
import frc.robot.subsystems.*;


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
    private final LimelightSubsystem s_Limelight;

    private Mode currentMode = Mode.Speaker;
    private boolean precise = false;
    private boolean fieldCentric = true;
    private double shooterSpeed = 0;

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
        s_Limelight = LimelightSubsystem.getInstance();

        s_Swerve.setDefaultCommand(
                new TeleopCommand(
                        driver.leftVerticalJoystick,
                        driver.leftHorizontalJoystick,
                        driver.rightHorizontalJoystick,
                        () -> false,
                        driver.leftJoystickButton
                )
        );

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        driver.leftBumper.whileTrue(new ExportCommand());
        driver.leftTrigger.whileTrue(new IntakeCommand());

        driver.rightBumper.whileTrue(new AimbotCommand());
        //driver.rightBumper.toggleOnTrue(new AimAssistCommand());
        driver.rightTrigger.whileTrue(new ShootCommand());

        driver.dUp.onTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        driver.dRight.onTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        driver.dLeft.whileTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.kRetractedAngle)));
        driver.dDown.whileTrue(new TransferCommand());

        driver.triangleButton.whileTrue(new SwerveAngleCommand(SwerveAngleCommand.SwerveAngle.ZERO));
        driver.squareButton.whileTrue(new InstantCommand(StateEstimator.getInstance()::resetPose));
        driver.circleButton.whileTrue(new InstantCommand(s_Swerve::resetOdometry));
        driver.crossButton.whileTrue(new InstantCommand(s_Limelight::toggleLeds)); // TODO: add on CANdle disabling
    }

    private void configureOperatorBindings() {
        operator.leftTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Speaker));
        // operator.leftTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

        operator.leftBumper.and(this::notClimberMode).whileTrue(new StartEndCommand(
                () -> s_Limelight.setLEDMode(LimelightSubsystem.LEDMode.BLINK),
                () -> s_Limelight.setLEDMode(LimelightSubsystem.LEDMode.OFF)
        ));
        // operator.leftBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeLeftClimberLocation(Constants.Climber.climberSpeed), s_Climber));

        operator.rightTrigger.and(this::notClimberMode).whileTrue(new InstantCommand(() -> currentMode = Mode.Amp));
        // operator.rightTrigger.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(-Constants.Climber.climberSpeed), s_Climber));

         operator.rightBumper.and(this::notClimberMode).whileTrue(new TransferCommand());
        // operator.rightBumper.and(this::climberMode).whileTrue(new RunCommand(() -> s_Climber.changeRightClimberLocation(Constants.Climber.climberSpeed), s_Climber));

        operator.dUp.whileTrue(new InstantCommand(this::increaseShooterPower, s_Shooter));
        operator.dDown.whileTrue(new InstantCommand(this::decreaseShooterPower, s_Shooter));
        operator.dLeft.whileTrue(new RunCommand(this::retractIntake, s_Intake));
        operator.dRight.whileTrue(new RunCommand(this::extendIntake, s_Intake));

        operator.triangleButton.toggleOnTrue(new StartEndCommand(() -> currentMode = Mode.Climb, () -> currentMode = Mode.Speaker));
        operator.squareButton.onTrue(new InstantCommand(() -> s_Intake.setSwivelPosition(Constants.Intake.kRetractedAngle)));
        operator.circleButton.whileTrue(new InstantCommand(this::togglePrecise));
        operator.crossButton.whileTrue(new SwerveAngleCommand(SwerveAngleCommand.SwerveAngle.CROSS));
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    public void updateTelemetry() {
        TelemetryUpdater.setTelemetryValue("ControlBoard/Target Shooter Speed", shooterSpeed);
        TelemetryUpdater.setTelemetryValue("ControlBoard/Mode", currentMode.toString());
    }

    private void togglePrecise() {
        precise = !precise;
    }

    public double shooterSpeedOffset() {
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
