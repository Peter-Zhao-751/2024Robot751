package frc.robot.commands.lowLevelCommands;
import frc.robot.Constants;
import frc.robot.commands.AimbotCommand;
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

    private final AimbotCommand aimbotCommand;

    private double startTime; // TODO: Maybe use WaitCommands instead?
    private double waitTime;
    private final boolean smartMode;
    private final boolean aimbotMode;
    private boolean started;

    private ControlBoard.Mode mode;
 
    public ShootCommand(boolean smartMode, boolean aimbotMode) {
        this.shooterSubsystem = ShooterSubsystem.getInstance();
        this.transferSubsystem = TransferSubsystem.getInstance();
        this.intakeSubsystem = IntakeSubsystem.getInstance();

        this.aimbotCommand = new AimbotCommand();

        this.smartMode = smartMode;
        this.aimbotMode = aimbotMode;

        addRequirements(shooterSubsystem, transferSubsystem, intakeSubsystem);
    }

    public ShootCommand() {
        this(true, true);
    }

    @Override
    public void initialize() {
        StateMachine.setState(StateMachine.State.Shoot);
        mode = ControlBoard.getInstance().getMode();
        startTime = System.currentTimeMillis();
        started = !smartMode;

        if (mode == ControlBoard.Mode.Speaker) {
            shooterSubsystem.setSpeed(ControlBoard.getInstance().shooterSpeed());
            waitTime = 1000;
            if (!smartMode) transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
            aimbotCommand.initialize();
        } else {
            intakeSubsystem.setSwivelPosition(Constants.Intake.kAmpAngle);
            waitTime = 1000;
            if (!smartMode) transferSubsystem.setTransferSpeed(-Constants.Transfer.intakeTransferSpeed);
        }
    }
    
    @Override
    public void execute() {
        if (smartMode && !started) {
            if (mode == ControlBoard.Mode.Speaker && shooterSubsystem.getShooterSpeed() > 0.95 * ControlBoard.getInstance().shooterSpeed()) {
                transferSubsystem.setTransferSpeed(Constants.Transfer.intakeTransferSpeed);
                startTime = System.currentTimeMillis();
                started = true;
            } else if (mode == ControlBoard.Mode.Amp && intakeSubsystem.getSwivelPosition() > 0.95 * Constants.Intake.kAmpAngle) {
                transferSubsystem.setTransferSpeed(-Constants.Transfer.intakeTransferSpeed);
                startTime = System.currentTimeMillis();
                started = true;
            }
        }
        aimbotCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setSpeed(0);
        transferSubsystem.setTransferSpeed(0);
        aimbotCommand.end(interrupted);
        intakeSubsystem.setSwivelPosition(Constants.Intake.kRetractedAngle);
        StateMachine.setState(StateMachine.State.Idle);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime > waitTime;
    }
}
