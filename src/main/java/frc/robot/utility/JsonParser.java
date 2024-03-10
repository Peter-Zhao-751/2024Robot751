package frc.robot.utility;

import java.util.ArrayList;
import java.util.Base64;
import java.io.File;
import java.io.FileReader;
import java.util.Iterator;

import frc.robot.commands.LowLevelCommands.IntakeCommand;
import frc.robot.commands.LowLevelCommands.ShootCommand;
import frc.robot.commands.LowLevelCommands.IntakeCommand.IntakeSwivelMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.commands.MoveCommand;

import edu.wpi.first.math.geometry.Translation2d;
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
    private static SwerveSubsystem swerveSubsystem;
    private static JSONObject jsonObject;
    private static JSONArray jsonArray;

    public JsonParser(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem){
        JsonParser.intakeSubsystem = intakeSubsystem;
        JsonParser.transferSubsystem = transferSubsystem;
        JsonParser.shooterSubsystem = shooterSubsystem;
        JsonParser.swerveSubsystem = swerveSubsystem;

        jsonObject = null;
        jsonArray = null; 
    }

    public ArrayList<Command> getAutonCommands(File pathFile) throws Exception{

        String encryptedData = new String(java.nio.file.Files.readAllBytes(pathFile.toPath()));

        String decryptedData = fullDecrypt(encryptedData);
        System.out.println(decryptedData);

        jsonObject = (JSONObject) new JSONParser().parse(decryptedData); 

        jsonArray = (JSONArray) jsonObject.get("points"); 

        ArrayList<Command> autonCommands = new ArrayList<>();

        Iterator<JSONObject> iterator = jsonArray.iterator();

        JSONObject point;

        while (iterator.hasNext()) {
            point = iterator.next();

            if (getEvent(point).isEmpty() || getEvent(point).equals("start")){
                ArrayList<Translation2d> interiorPoints = new ArrayList<>();
                while (iterator.hasNext()){
                    JSONObject interiorPoint = iterator.next();
                    if (getEvent(interiorPoint).isEmpty()){
                        interiorPoints.add(getInteriorPoint(interiorPoint));
                    }else{
                        point = interiorPoint;
                        break;
                    }
                }

                MoveCommand newMovementCommand = new MoveCommand(swerveSubsystem, getMainPoint(point), interiorPoints);

                double delay = newMovementCommand.getETA();

                switch (getEvent(point)){
                    case "Shoot":
                        //double primeDelay = (delay-Constants.Shooter.spinUpTime) > 0 ? (delay-Constants.Shooter.spinUpTime) : 0;
                        //ParallelDeadlineGroup moveAndPrime = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(primeDelay), new InstantCommand()));
                        autonCommands.add(new SequentialCommandGroup(newMovementCommand, new ShootCommand(shooterSubsystem, transferSubsystem, 200, true)));
                        break;
                    case "Pickup":
                        double intakeDelay = (delay-5) > 0 ? (delay-5) : 0;
                        ParallelDeadlineGroup moveAndIntake = new ParallelDeadlineGroup(newMovementCommand, new SequentialCommandGroup(new WaitCommand(intakeDelay), new IntakeCommand(intakeSubsystem, transferSubsystem, IntakeSwivelMode.Extend, true)));
                        autonCommands.add(moveAndIntake);
                        break;
                    default:
                        autonCommands.add(newMovementCommand);
                        break;
                }
            }
        }
        System.out.println(autonCommands);
        return autonCommands;
    }

    private static String shift(String inStr, int x, boolean de) {
        if (inStr.isEmpty() || x == 0) return inStr;

        x = x % inStr.length();
        if (x < 0) x += inStr.length();

        if (de) return inStr.substring(x) + inStr.substring(0, x);
        else return inStr.substring(inStr.length() - x) + inStr.substring(0, inStr.length() - x);
    }

    private static String base64Decode(String str) {
        byte[] decodedBytes = Base64.getDecoder().decode(str);
        return new String(decodedBytes);
    }

    private static String fullDecrypt(String data) {
        String temp = data;
        temp = temp.replace("<barn2path> ", "").replace(" <barn2path>", "")
                   .replace("$", "A").replace("!", "M").replace("@", "C").replace("#", "D")
                   .replace("&", "w").replace("*", "j").replace("^", "I").replace("%", "i")
                   .replace("<", "g").replace(">", "k").replace("?", "S");
        temp = shift(temp, 751, true);
        temp = base64Decode(temp);
        return temp;
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
