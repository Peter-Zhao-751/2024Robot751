package frc.robot.utility;
import java.util.ArrayList;
import java.util.Comparator;

import frc.robot.Constants;
import frc.robot.subsystems.Component;

public class CurrentManager {
    public static final double maxCurrent = Constants.CurrentManager.maxCurrent;
    public static final double maxPercent = Constants.CurrentManager.maxPercent;
    public static final double nominalPercent = Constants.CurrentManager.nominalPercent;
    public static double totalCurrent = 0;

    private static final ArrayList<Component> components = new ArrayList<>();

    private CurrentManager() {
        throw new UnsupportedOperationException("CurrentManager class cannot be instantiated");
    }

    public static void addComponent(Component component) {
        components.add(component);
    }

    public void allocateCurrent() {
        components.sort(Comparator.comparingInt(Component::getPriority));
        double totalAvailableCurrent = maxCurrent*maxPercent;

        double totalRequested = components.stream().mapToDouble(Component::getCurrentDraw).sum();
        double allocationRatio = totalAvailableCurrent / totalRequested;
        double remainingCurrent = totalAvailableCurrent;

        if (allocationRatio >= 1) {
            // Enough current for all requests
            components.forEach(c -> c.allocateCurrent(c.getCurrentDraw()));
            return;
        }

        // Allocate minimum possible current to all components
        int maxPriority = 0;
        int totalPriority = 0;
        for (Component component : components) {
            remainingCurrent -= component.getCurrentDraw() / allocationRatio * 0.8;
            totalPriority += component.getPriority();
            maxPriority = Math.max(maxPriority, component.getPriority());
        }

        double noneFinalRemainingCurrent = remainingCurrent;
            
        for (Component component : components) {
            double additionalNeeded = component.getCurrentDraw() - (component.getCurrentDraw() * allocationRatio * 0.8);
            double additionalFairShare = (double) (maxPriority - component.getPriority()) / totalPriority * noneFinalRemainingCurrent;
            double finalAdditionalAllocated = Math.min(additionalNeeded, additionalFairShare);
            component.allocateCurrent(component.getCurrentDraw() * allocationRatio + finalAdditionalAllocated);
            remainingCurrent -= finalAdditionalAllocated;
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
