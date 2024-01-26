package frc.robot.subsystems;

import java.util.ArrayList;
import java.io.FileReader; 
import java.util.Iterator; 
import java.util.Map;

import frc.robot.Constants;
import frc.robot.commands.AutonCommandSegment;
import frc.robot.commands.MoveToLocation;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.Shooter;
import frc.robot.commands.Intake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
  
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

    public ArrayList<AutonCommandSegment> getAutonCommands(String fileName) throws Exception{

        jsonObject = (JSONObject) new JSONParser().parse(new FileReader(fileName+".json"));  
        jsonArray = (JSONArray) jsonObject.get("waypoints"); 


        ArrayList<AutonCommandSegment> autonCommands = new ArrayList<AutonCommandSegment>();
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

                MoveToLocation newMovementCommand = new MoveToLocation(swerveSubsystem, newLocation, interiorPoints);

                switch ( (String) point.get("e")){
                    case "Shoot": 
                        double delay = (newMovementCommand.ETA() - Constants.shooter.spinUpTime) > 0 ? newMovementCommand.ETA() - Constants.shooter.spinUpTime : 0;
                        ParallelCommandGroup moveAndPrime = new ParallelCommandGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(delay), new SpinShooter()));
                        autonCommands.add(new AutonCommandSegment(moveAndPrime, new Shooter(shooterSubsystem), "Shoot"));
                        break;
                    case "Pickup":
                        autonCommands.add(new AutonCommandSegment(newMovementCommand, new Intake(intakeSubsystem), "Pickup"));
                        break;
                    default:
                        autonCommands.add(new AutonCommandSegment(newMovementCommand));
                        break;
                }
            } else{
                point = iterator.next();
            }
        }
        return autonCommands;
    }

    public Translation2d getInteriorPoint(JSONObject point){
        return new Translation2d((double)point.get("x"), (double)point.get("y"));
    }

    public Pose2d getMainPoint(JSONObject point){
        return new Pose2d((double)point.get("x"), (double)point.get("y"), new Rotation2d((double)point.get("r")));
    }
}
