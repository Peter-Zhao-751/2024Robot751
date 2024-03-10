package frc.robot.utility;

import java.util.ArrayList;
import java.util.Base64;
import java.io.File;
import java.io.FileReader;
import java.nio.charset.StandardCharsets;
import java.util.Iterator; 

import frc.robot.commands.lowLevelCommands.IntakeCommand;
import frc.robot.commands.lowLevelCommands.ShootCommand;
import frc.robot.commands.lowLevelCommands.IntakeCommand.IntakeSwivelMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.commands.MoveCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

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

    public static void JsonParser(IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem){
        JsonParser.intakeSubsystem = intakeSubsystem;
        JsonParser.transferSubsystem = transferSubsystem;
        JsonParser.shooterSubsystem = shooterSubsystem;
        JsonParser.swerveSubsystem = swerveSubsystem;

        jsonObject = null;
        jsonArray = null; 
    }

    public static ArrayList<Command> getAutonCommands(File pathFile) throws Exception{

        String encryptedData = new String(java.nio.file.Files.readAllBytes(pathFile.toPath()));

        String decryptedData = fullDecrypt(encryptedData);
        System.out.println(decryptedData);

        jsonObject = (JSONObject) new JSONParser().parse(decryptedData); 

        jsonArray = (JSONArray) jsonObject.get("points"); 

        ArrayList<Command> autonCommands = new ArrayList<>();

        Iterator<JSONObject> iterator = jsonArray.iterator();

        JSONObject point = null;

        while (iterator.hasNext()) {
            if (point == null){
                point = iterator.next();
            }

            if (getEvent(point).isEmpty() || getEvent(point).equals("start")){
                Pose2d newLocation = null;

                ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
                while (iterator.hasNext()){
                    JSONObject interiorPoint = iterator.next();
                    if (getEvent(interiorPoint).isEmpty()){
                        interiorPoints.add(getInteriorPoint(interiorPoint));
                    }else{
                        newLocation = getMainPoint(interiorPoint);
                        point = interiorPoint;
                        break;
                    }
                }

                MoveCommand newMovementCommand = new MoveCommand(swerveSubsystem, newLocation, interiorPoints);

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
            if (iterator.hasNext()) point = iterator.next();
        }
        System.out.println(autonCommands);
        return autonCommands;
    }

    public static String shfff(String inStr, int x, boolean de) {
        if (inStr.isEmpty() || x == 0) {
            return inStr;
        }

        x = x % inStr.length();
        if (x < 0) {
            x += inStr.length();
        }

        if (de) {
            return inStr.substring(x) + inStr.substring(0, x);
        } else {
            return inStr.substring(inStr.length() - x) + inStr.substring(0, inStr.length() - x);
        }
    }

    public static String base64Decode(String base64String) {
        byte[] decodedBytes = Base64.getDecoder().decode(base64String);
        return new String(decodedBytes, StandardCharsets.UTF_8);
    }

    public static String base64Encode(String str) {
        return Base64.getEncoder().encodeToString(str.getBytes(StandardCharsets.UTF_8));
    }

    public static String fullEncrypt(String data) {
        String temp = data;
        temp = base64Encode(temp);
        temp = shfff(temp, 751, false);
        temp = temp.replace("A", "$").replace("M", "!").replace("C", "@").replace("D", "#").replace("w", "&").replace("j", "*").replace("I", "^").replace("i", "%").replace("g", "<").replace("k", ">").replace("S", "?");
        temp = "<barn2path> " + temp + " </barn2path>";
        return temp;
    }

    public static String fullDecrypt(String data) {
        String temp = data;
        temp = temp.replaceAll("\\s*</?barn2path>\\s*", "");
        temp = temp.replace("$", "A").replace("!", "M").replace("@", "C").replace("#", "D")
                   .replace("&", "w").replace("*", "j").replace("^", "I").replace("%", "i")
                   .replace("<", "g").replace(">", "k").replace("?", "S");
        temp = shfff(temp, 751, true);
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
