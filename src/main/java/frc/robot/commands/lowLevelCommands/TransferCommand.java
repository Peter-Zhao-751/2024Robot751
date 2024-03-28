package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;


public class TransferCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private ControlBoard.Mode currentMode;

    public TransferCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.transferSubsystem = TransferSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(this.transferSubsystem, this.intakeSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        currentMode = ControlBoard.getInstance().getMode();

        if (currentMode == ControlBoard.Mode.Speaker) {
            transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed/10);
        } else {
            intakeSubsystem.setIntakeSpeed(Constants.Intake.intakeSpeed/10);
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
        if (currentMode == ControlBoard.Mode.Speaker) {
            transferSubsystem.setTransferSpeed(0);
        } else {
            intakeSubsystem.setIntakeSpeed(0);
        }
        StateMachine.setState(StateMachine.State.Idle);
    }
}
