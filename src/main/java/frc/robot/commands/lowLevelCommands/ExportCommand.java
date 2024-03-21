package frc.robot.commands.lowLevelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;

public class ExportCommand extends Command {
    private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private ControlBoard.Mode mode;

    public ExportCommand() {
        transferSubsystem = TransferSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        addRequirements(transferSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        transferSubsystem.setTransferSpeed(-50);
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
