package frc.robot.subsystems;
import frc.robot.Constants;

public class CurrentManager {
    private static double tempCurrent = 0;
    private static boolean hasSetDriveTrain = false;
    private static boolean hasSetShooter = false;
    private static boolean hasSetIntake = false;
    private static boolean hasSetClimber = false;

    public static double totalCurrent = 0;

    static enum Subsystem {
        DriveTrain,
        Shooter,
        Intake,
        Climber
    }

    public static final double maxCurrent = Constants.CurrentManager.maxCurrent;
    public static final double maxPercent = Constants.CurrentManager.maxPercent;
    public static final double nominalPercent = Constants.CurrentManager.nominalPercent;

    public static void updateCurrent(double current, Subsystem part){
        if(part == Subsystem.DriveTrain && !hasSetDriveTrain){
            hasSetDriveTrain = true;
            tempCurrent += current;
        }
        else if(part == Subsystem.Shooter && !hasSetShooter){
            hasSetShooter = true;
            tempCurrent += current;
        }
        else if(part == Subsystem.Intake && !hasSetIntake){
            hasSetIntake = true;
            tempCurrent += current;
        }
        else if(part == Subsystem.Climber && !hasSetClimber){
            hasSetClimber = true;
            tempCurrent += current;
        }

        if (hasSetClimber && hasSetDriveTrain && hasSetIntake && hasSetShooter ){
            hasSetClimber = false;
            hasSetDriveTrain = false;
            hasSetIntake = false;
            hasSetShooter = false;
            totalCurrent = tempCurrent;
            tempCurrent = 0;
        }
    }
    public static double getCurrent(){
        return totalCurrent;
    }
    public static double getPercent(){
        return totalCurrent / maxCurrent;
    }
    public static boolean isOverNominal(){
        return getPercent() > nominalPercent;
    }
    public static boolean isOverMax(){
        return getPercent() > maxPercent;
    }
}
