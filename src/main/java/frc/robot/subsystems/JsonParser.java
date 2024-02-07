package frc.robot.subsystems;

import java.util.ArrayList;
import java.io.FileReader; 
import java.util.Iterator; 

import frc.robot.Constants;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.Shooter;
import frc.robot.commands.Intake;
import frc.robot.commands.Move;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
  
import org.json.simple.JSONArray; 
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class JsonParser {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SwerveDrive swerveSubsystem;
    private JSONObject jsonObject;
    private JSONArray jsonArray;

    public JsonParser(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, SwerveDrive swerveSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        jsonObject = null;
        jsonArray = null; 
    }

    public ArrayList<Command> getAutonCommands(String fileName) throws Exception{

        jsonObject = (JSONObject) new JSONParser().parse(new FileReader(fileName+".json"));  
        jsonArray = (JSONArray) jsonObject.get("waypoints"); 


        ArrayList<Command> autonCommands = new ArrayList<>();
        Iterator<JSONObject> iterator = jsonArray.iterator(); 

        JSONObject point = null;

        while (iterator.hasNext()) {
            if (point == null){
                point = iterator.next();
            }

            if (!point.get("e").equals("")){
                
                Pose2d newLocation = null;
                
                ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
                while (iterator.hasNext()){
                    JSONObject interiorPoint = iterator.next();
                    if (interiorPoint.get("e").equals("")){
                        interiorPoints.add(getInteriorPoint(interiorPoint));
                    }else{
                        newLocation = getMainPoint(interiorPoint);
                        point = interiorPoint;
                        break;
                    }
                }

                Move newMovementCommand = new Move(swerveSubsystem, newLocation, interiorPoints);

                double delay = newMovementCommand.getETA();

                switch ( (String) point.get("e")){
                    case "Shoot": 
                        double primeDelay = (delay-Constants.shooter.spinUpTime) > 0 ? (delay-Constants.shooter.spinUpTime) : 0;
                        ParallelDeadlineGroup moveAndPrime = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(primeDelay), new SpinShooter()));
                        autonCommands.add(new SequentialCommandGroup(moveAndPrime, new Shooter(shooterSubsystem)));
                        break;
                    case "Pickup":
                        double intakeDelay = (delay-5) > 0 ? (delay-5) : 0;
                        ParallelDeadlineGroup moveAndIntake = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(intakeDelay), new Intake(intakeSubsystem)));
                        autonCommands.add(moveAndIntake);
                        break;
                    default:
                        autonCommands.add(newMovementCommand);
                        break;
                }

            }
            point = iterator.next();
        }
        return autonCommands;
    }

    private Translation2d getInteriorPoint(JSONObject point){
        return new Translation2d((double)point.get("x"), (double)point.get("y"));
    }

    private Pose2d getMainPoint(JSONObject point){
        return new Pose2d((double)point.get("x"), (double)point.get("y"), new Rotation2d((double)point.get("r")));
    }
}
