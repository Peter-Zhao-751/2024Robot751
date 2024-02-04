package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class AutonCommandSegment extends SequentialCommandGroup{
    public AutonCommandSegment(Command locationCommand, SequentialCommandGroup action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(locationCommand, action);
    }
    public AutonCommandSegment(Command locationCommand,  Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(locationCommand, action);
    }
    public AutonCommandSegment(ParallelCommandGroup parallelCommand, Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(parallelCommand, action);
    }
    public AutonCommandSegment(ParallelDeadlineGroup parallelCommand, Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(parallelCommand, action);
    }
    public AutonCommandSegment(Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(action);
    }
    public AutonCommandSegment(Command locationCommand){
        addCommands(locationCommand);
    }
    public AutonCommandSegment(ParallelCommandGroup action){
        addCommands(action);
    }
    public AutonCommandSegment(SequentialCommandGroup action){
        addCommands(action);
    }
}
