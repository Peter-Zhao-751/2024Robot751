package frc.robot.commands.lowLevelCommands;

import frc.robot.commands.lowLevelCommands.Transfer.TransferMode;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;

public class Intake extends Command {

    public enum IntakeSwivelMode {
        Extend(Constants.Intake.kSwivelExtendedAngle),
        Retract(Constants.Intake.kSwivelRetractedAngle),
        Maintenance(Constants.Intake.kSwivelMaintenanceAngle);

        public double desiredAngle;

        private IntakeSwivelMode(double desiredAngle) {
            this.desiredAngle = desiredAngle;
        }
    }
    private IntakeSubsystem intakeSubsystem;
    private Transfer transferCommand;

    private IntakeSwivelMode desiredSwivelState;

    public Intake(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, IntakeSwivelMode extend, boolean smartMode) {
        this.desiredSwivelState = extend;
        this.intakeSubsystem = intakeSubsystem;
        this.transferCommand = new Transfer(20, transferSubsystem, TransferMode.Intake, smartMode); // TODO: Tune this speed, it is in centimeters per second
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Intake);
        intakeSubsystem.setSwivelPosition(desiredSwivelState.desiredAngle);
        if (desiredSwivelState == IntakeSwivelMode.Extend) {
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
        transferCommand.end(interrupted);
        StateMachine.setState(StateMachine.State.Idle);
    }
    
    @Override
    public boolean isFinished() {
        return desiredSwivelState == IntakeSwivelMode.Retract || desiredSwivelState == IntakeSwivelMode.Maintenance || transferCommand.isFinished();
    }
}
