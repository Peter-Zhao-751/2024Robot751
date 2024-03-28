package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.StupidAimAssistCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;


public class SpinShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
   private final IntakeSubsystem intakeSubsystem;

    private ControlBoard.Mode mode;
    private final StupidAimAssistCommand aimAssistCommand;

    public SpinShooterCommand() {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.aimAssistCommand = new StupidAimAssistCommand();
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
//            shooterSubsystem.setSpeed(ControlBoard.getInstance().shooterSpeed());
            intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
            aimAssistCommand.initialize();
        } else {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
            shooterSubsystem.setSpeed(0);
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.
     */
    @Override
    public void execute() {
        aimAssistCommand.execute();
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
//        return false;
        return aimAssistCommand.isFinished();
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
        if (mode == ControlBoard.Mode.Speaker) aimAssistCommand.end(interrupted);
        StateMachine.setState(StateMachine.State.Idle);
    }
}
