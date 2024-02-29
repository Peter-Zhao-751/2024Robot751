package frc.robot.commands.lowLevelCommands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;

public class Intake extends Command {

    public enum IntakeSwivelState {
        Extended(Constants.Intake.kSwivelExtendedAngle),
        Retracted(Constants.Intake.kSwivelRetractedAngle),
        Maintenance(Constants.Intake.kSwivelMaintenanceAngle);

        public double desiredAngle;

        private IntakeSwivelState(double desiredAngle) {
            this.desiredAngle = desiredAngle;
        }
    }
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;

    private Transfer transferCommand;

    private IntakeSwivelState desiredSwivelState;

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IntakeSwivelState extend, boolean smartMode) {
        this.desiredSwivelState = extend;
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.transferCommand = new Transfer(0.2, transferSubsystem, smartMode); // TODO: Change speed it is in m/s
        addRequirements(intakeSubsystem);
    }

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IntakeSwivelState extend) {
        this(intakeSubsystem, transferSubsystem, extend, false);
    }

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem) {
        this(intakeSubsystem, transferSubsystem, IntakeSwivelState.Retracted, false);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        intakeSubsystem.setSwivelPosition(desiredSwivelState.desiredAngle);
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
        return desiredSwivelState == IntakeSwivelState.Retracted || desiredSwivelState == IntakeSwivelState.Maintenance || transferCommand.isFinished();
    }
}
