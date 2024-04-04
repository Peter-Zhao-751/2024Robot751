package frc.robot.utility;

import java.util.ArrayList;
import java.util.Base64;
import java.io.File;
import java.io.FileReader;
import java.util.Iterator;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.gamepieceCommands.IntakeCommand;
import frc.robot.commands.gamepieceCommands.ShootCommand;
import frc.robot.commands.movementCommands.AimbotCommand;
import frc.robot.commands.movementCommands.MoveCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class Barn2PathInterpreter {
    private static JSONObject jsonObject;
    private static JSONArray jsonArray;

    public Barn2PathInterpreter(){
        jsonObject = null;
        jsonArray = null;
    }

    public ArrayList<Command> getAutonCommands(File pathFile) throws Exception{

        String encryptedData = new String(java.nio.file.Files.readAllBytes(pathFile.toPath()));

        jsonObject = (JSONObject) new JSONParser().parse(encryptedData);

        jsonArray = (JSONArray) jsonObject.get("points");

        ArrayList<Command> autonCommands = new ArrayList<>();

        Iterator iterator = jsonArray.iterator();

        JSONObject point;

        System.out.println(jsonArray.toString());

        while (iterator.hasNext()) {
            point = (JSONObject) iterator.next();

            if (getEvent(point).isEmpty() || getEvent(point).equals("start")){
                ArrayList<Translation2d> interiorPoints = new ArrayList<>();
                while (iterator.hasNext()){
                    JSONObject interiorPoint = (JSONObject) iterator.next();
                    if (getEvent(interiorPoint).isEmpty()){
                        interiorPoints.add(getInteriorPoint(interiorPoint));
                    }else{
                        point = interiorPoint;
                        break;
                    }
                }



                switch (getEvent(point).toLowerCase()){
					case "shoot":
						MoveCommand newShootMovementCommand = new MoveCommand(getMainPoint(point), interiorPoints);
                        autonCommands.add(new SequentialCommandGroup(newShootMovementCommand, new AimbotCommand(), new ShootCommand(true)));
                        break;
					case "intake":
						MoveCommand newIntakeMovementCommand = new MoveCommand(getMainPoint(point), interiorPoints);
						double delay = newIntakeMovementCommand.getETA();
                        double intakeDelay = (delay-5) > 0 ? (delay-5) : 0;
                        ParallelDeadlineGroup moveAndIntake = new ParallelDeadlineGroup(newIntakeMovementCommand, new SequentialCommandGroup(new WaitCommand(intakeDelay), new IntakeCommand()));
                        autonCommands.add(moveAndIntake);
                        break;
					default:
						MoveCommand newMovementCommand = new MoveCommand(getMainPoint(point), interiorPoints);
                        autonCommands.add(newMovementCommand);
                        break;
                }
            }
        }
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
        System.err.println(str);
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
                // pretty sure this doesn't work
                String data = new String(java.nio.file.Files.readAllBytes(pathFile.toPath()));
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
