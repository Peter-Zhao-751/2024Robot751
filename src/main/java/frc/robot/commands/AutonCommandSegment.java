package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutonCommandSegment extends SequentialCommandGroup{
    public AutonCommandSegment(MoveToLocation locationCommand, SequentialCommandGroup action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(locationCommand, action);
    }
    public AutonCommandSegment(MoveToLocation locationCommand,  Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(locationCommand, action);
    }
    public AutonCommandSegment(ParallelCommandGroup parallelCommand, Command action, String name){
        SmartDashboard.putString("Current Command", name);
        addCommands(parallelCommand, action);
    }
    public AutonCommandSegment(MoveToLocation locationCommand){
        addCommands(locationCommand);
    }
    public AutonCommandSegment(ParallelCommandGroup action){
        addCommands(action);
    }
    public AutonCommandSegment(SequentialCommandGroup action){
        addCommands(action);
    }
}
