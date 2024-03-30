package frc.robot.commands.nonMovementCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private ControlBoard.Mode currentMode;

    public IntakeCommand() {
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.transferSubsystem = TransferSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        currentMode = ControlBoard.getInstance().getMode();

        if (currentMode == ControlBoard.Mode.Speaker) {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kIntakeAngle);
            transferSubsystem.setIntakeTransfer(Constants.Transfer.intakeTransferSpeed);
            transferSubsystem.setShooterTransfer(-50);
        } else if (currentMode == ControlBoard.Mode.Amp) {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kIntakeAngle);
        }
    }

    @Override
    public void execute() {
        if (intakeSubsystem.closeToSetpoint()) {
            intakeSubsystem.setIntakeSpeed(Constants.Intake.intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        transferSubsystem.stop();
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return switch (currentMode) {
            case Speaker -> transferSubsystem.beamBroken();
            case Amp -> intakeSubsystem.beamBroken();
            default -> false;
        };
    }
}
