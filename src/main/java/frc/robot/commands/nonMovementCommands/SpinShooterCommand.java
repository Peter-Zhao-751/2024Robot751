package frc.robot.commands.nonMovementCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.movementCommands.AimAssistCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;


public class SpinShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    private ControlBoard.Mode mode;
    private final AimAssistCommand aimAssistCommand;

    public SpinShooterCommand() {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.limelightSubsystem = LimelightSubsystem.getInstance();

        this.aimAssistCommand = new AimAssistCommand();
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();

        if (mode == ControlBoard.Mode.Speaker) {
            shooterSubsystem.setSpeed(ControlBoard.getInstance().shooterSpeed());
            intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);

            StateMachine.setState(StateMachine.State.Aimbot);

            limelightSubsystem.setVisionMode();
            limelightSubsystem.setLEDMode(LimelightSubsystem.LEDMode.ON);
        } else {
            StateMachine.setState(StateMachine.State.Shoot);

            shooterSubsystem.setSpeed(0);
            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.
     */
    @Override
    public void execute() {
        if (mode == ControlBoard.Mode.Speaker) {
            if (!aimAssistCommand.isScheduled() && limelightSubsystem.hasTarget()) {
                aimAssistCommand.schedule();
            } else {
                aimAssistCommand.execute();
            }
        }
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);

        limelightSubsystem.setDriverMode();
        limelightSubsystem.setLEDMode(LimelightSubsystem.LEDMode.OFF);

        if (mode == ControlBoard.Mode.Speaker) aimAssistCommand.end(interrupted);
        StateMachine.setState(StateMachine.State.Idle);
    }
}
