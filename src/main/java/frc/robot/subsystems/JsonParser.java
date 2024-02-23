package frc.robot.subsystems;

import java.util.ArrayList;
import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.Iterator; 

import frc.robot.Constants;
import frc.robot.commands.Shoot;
import frc.robot.commands.Intake;
import frc.robot.commands.Move;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.json.simple.JSONArray; 
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class JsonParser {
    private static IntakeSubsystem intakeSubsystem;
    private static TransferSubsystem transferSubsystem;
    private static ShooterSubsystem shooterSubsystem;
    private static SwerveDrive swerveSubsystem;
    private static JSONObject jsonObject;
    private static JSONArray jsonArray;

    public static void JsonParser(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, SwerveDrive swerveSubsystem){
        JsonParser.intakeSubsystem = intakeSubsystem;
        JsonParser.transferSubsystem = transferSubsystem;
        JsonParser.shooterSubsystem = shooterSubsystem;
        JsonParser.swerveSubsystem = swerveSubsystem;

        jsonObject = null;
        jsonArray = null; 
    }

    public static ArrayList<Command> getAutonCommands(File pathFile) throws Exception{

        jsonObject = (JSONObject) new JSONParser().parse(new FileReader(pathFile)); 

        jsonArray = (JSONArray) jsonObject.get("points"); 

        ArrayList<Command> autonCommands = new ArrayList<>();

        Iterator<JSONObject> iterator = jsonArray.iterator(); 

        JSONObject point = null;

        while (iterator.hasNext()) {
            if (point == null){
                point = iterator.next();
            }
            
            if (getEvent(point).equals("") || getEvent(point).equals("start")){
                Pose2d newLocation = null;
                
                ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
                while (iterator.hasNext()){
                    JSONObject interiorPoint = iterator.next();
                    if (getEvent(interiorPoint).equals("")){
                        interiorPoints.add(getInteriorPoint(interiorPoint));
                    }else{
                        newLocation = getMainPoint(interiorPoint);
                        point = interiorPoint;
                        break;
                    }
                }

                Move newMovementCommand = new Move(swerveSubsystem, newLocation, interiorPoints);

                double delay = newMovementCommand.getETA();

                switch (getEvent(point)){
                    case "Shoot": 
                        double primeDelay = (delay-Constants.Shooter.spinUpTime) > 0 ? (delay-Constants.Shooter.spinUpTime) : 0;
                        ParallelDeadlineGroup moveAndPrime = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(primeDelay), new InstantCommand()));
                        autonCommands.add(new SequentialCommandGroup(moveAndPrime, new Shoot(shooterSubsystem, transferSubsystem)));
                        break;
                    case "Pickup":
                        double intakeDelay = (delay-5) > 0 ? (delay-5) : 0;
                        ParallelDeadlineGroup moveAndIntake = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(intakeDelay), new Intake(intakeSubsystem, transferSubsystem)));
                        autonCommands.add(moveAndIntake);
                        break;
                    default:
                        autonCommands.add(newMovementCommand);
                        break;
                }
            }
            if (iterator.hasNext()) point = iterator.next();
        }
        System.out.println(autonCommands);
        return autonCommands;
    }

    public static String getAutonPreview(File pathFile){
        if (pathFile != null){
            try { 
                jsonObject = (JSONObject) new JSONParser().parse(new FileReader(pathFile)); 
                return (String) jsonObject.get("preview");
            }     
            catch (Exception e) { System.out.println("Error: " + e);}
        }
        return "No Path";
    }

    
    private static Translation2d getInteriorPoint(JSONObject point){
        return new Translation2d((double)point.get("x"), (double)point.get("y"));
    }

    private static String getEvent(JSONObject point){
        return (String) point.get("e");
    }

    private static Pose2d getMainPoint(JSONObject point){
        return new Pose2d((double)point.get("x"), (double)point.get("y"), new Rotation2d((double)point.get("r")));
    }
}
