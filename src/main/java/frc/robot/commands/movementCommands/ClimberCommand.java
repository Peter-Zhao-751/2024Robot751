package frc.robot.commands.movementCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    public enum Direction {
        Fwd,
        Bwd
    }

    public enum Side {
        Left,
        Right
    }

    private final Direction dir;
    private final Side side;
    private final ClimberSubsystem climberSubsystem;

    public ClimberCommand(Direction dir, Side side) {
        this.dir = dir;
        this.side = side;

        this.climberSubsystem = ClimberSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        if (side == Side.Left) {
            climberSubsystem.setLeftVoltage(dir == Direction.Fwd ? -Constants.Climber.voltage : Constants.Climber.voltage);
        } else {
            climberSubsystem.setRightVoltage(dir == Direction.Fwd ? -Constants.Climber.voltage : Constants.Climber.voltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (side == Side.Left) {
            climberSubsystem.setLeftVoltage(0);
        } else {
            climberSubsystem.setRightVoltage(0);
        }
    }
}
