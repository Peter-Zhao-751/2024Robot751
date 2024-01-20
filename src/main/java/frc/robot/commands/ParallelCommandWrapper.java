package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ParallelCommandWrapper extends ParallelCommandGroup{
    public ParallelCommandWrapper(SequentialCommandGroup action1, SequentialCommandGroup action2){
        addCommands(action1, action2);
    }
}
