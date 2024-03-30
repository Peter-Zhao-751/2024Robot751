package frc.robot.commands.lowLevelCommands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.utility.ControlBoard;
import frc.robot.utility.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command{
    private final ShooterSubsystem shooterSubsystem;
   private final TransferSubsystem transferSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private final boolean smartMode;
    private boolean started;

    private ControlBoard.Mode mode;

    public ShootCommand(boolean smartMode) {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.transferSubsystem = TransferSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.smartMode = smartMode;

        addRequirements(shooterSubsystem, transferSubsystem, intakeSubsystem);
    }

    public ShootCommand() {
        this(true);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();
        started = !smartMode;

        if (mode == ControlBoard.Mode.Speaker) {
            shooterSubsystem.setSpeed(ControlBoard.getInstance().shooterSpeed());
            if (!smartMode) transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
        } else {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
            if (!smartMode) intakeSubsystem.setIntakeSpeed(-Constants.Shooter.intakeAmpSpeed);
        }
    }

    @Override
    public void execute() {
        if (!started && smartMode) {
            if (mode == ControlBoard.Mode.Speaker && shooterSubsystem.getShooterSpeed() > 0.95 * ControlBoard.getInstance().shooterSpeed()) {
                transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
                started = true;
            } else if (mode == ControlBoard.Mode.Amp && intakeSubsystem.closeToSetpoint()) {
                intakeSubsystem.setIntakeSpeed(-Constants.Shooter.intakeAmpSpeed);
                started = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
        transferSubsystem.setTransferSpeed(0);
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        StateMachine.setState(StateMachine.State.Idle);
    }
}
