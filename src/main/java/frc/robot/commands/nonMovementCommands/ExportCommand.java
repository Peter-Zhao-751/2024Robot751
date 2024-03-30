package frc.robot.commands.nonMovementCommands;

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
        if (mode == ControlBoard.Mode.Speaker) {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
            transferSubsystem.setTransferSpeed(-50);
        } else if (mode == ControlBoard.Mode.Amp) {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kIntakeAngle);
            intakeSubsystem.setIntakeSpeed(-10);
        }
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.stop();
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        StateMachine.setState(StateMachine.State.Idle);
    }
}
