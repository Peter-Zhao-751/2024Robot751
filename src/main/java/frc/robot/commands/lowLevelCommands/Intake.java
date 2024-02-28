package frc.robot.commands.lowLevelCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;

public class Intake extends Command {

    public enum IntakeSwivelState {
        Extended,
        Retracted,
        Maintenance
    }
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;

    private Transfer transferCommand;

    private double startTime;

    private IntakeSwivelState desiredSwivelState;

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IntakeSwivelState extend) {
        this.desiredSwivelState = extend;
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.transferCommand = new Transfer(0.2, transferSubsystem); // TODO: Change speed it is in m/s
        addRequirements(intakeSubsystem);
    }

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this(intakeSubsystem, transferSubsystem, IntakeSwivelState.Retracted);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        startTime = System.currentTimeMillis();
        double swivelAngle;
        switch (desiredSwivelState) {
            case Extended -> swivelAngle = Constants.Intake.kSwivelExtendedAngle;
            case Maintenance -> swivelAngle = Constants.Intake.kSwivelMaintenanceAngle;
            default -> swivelAngle = Constants.Intake.kSwivelRetractedAngle;
        }
        intakeSubsystem.setSwivelPosition(swivelAngle);
        if (desiredSwivelState == IntakeSwivelState.Extended) {
            transferCommand.schedule();
        }
    }

    @Override
    public void execute() {
        // The actual motor control is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSwivelPosition(Constants.Intake.kSwivelRetractedAngle);
        transferSubsystem.setIntakeTransfer(0);
        StateMachine.setState(StateMachine.State.Idle);
    }
    @Override
    public boolean isFinished() {
        return transferSubsystem.beamBroken() || System.currentTimeMillis() - startTime >= Constants.Transfer.maxTransferTime * 1000 || desiredSwivelState == IntakeSwivelState.Retracted || desiredSwivelState == IntakeSwivelState.Maintenance;
    }
}
