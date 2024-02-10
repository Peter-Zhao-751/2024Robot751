package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Comparator;

import frc.robot.Constants;

public class CurrentManager {
    public static final double maxCurrent = Constants.CurrentManager.maxCurrent;
    public static final double maxPercent = Constants.CurrentManager.maxPercent;
    public static final double nominalPercent = Constants.CurrentManager.nominalPercent;
    public static double totalCurrent = 0;

    private static ArrayList<Component> components = new ArrayList<Component>();

    public static void addComponent(Component component) {
        components.add(component);
    }

    public void allocateCurrent() {
        components.sort(Comparator.comparingInt(Component::getPriority));
        double totalAvailableCurrent = maxCurrent*maxPercent;

        double totalRequested = components.stream().mapToDouble(Component::getRequestedCurrent).sum();
        double allocationRatio = totalAvailableCurrent / totalRequested;
        double remainingCurrent = totalAvailableCurrent;

        if (allocationRatio >= 1) {
            // Enough current for all requests
            components.forEach(c -> c.allocateCurrent(c.getRequestedCurrent()));
            return;
        }

        // Allocate minimum possible current to all components
        int maxPriority = 0;
        int totalPriority = 0;
        for (Component component : components) {
            remainingCurrent -= component.getRequestedCurrent() / allocationRatio * 0.8;
            totalPriority += component.getPriority();
            maxPriority = Math.max(maxPriority, component.getPriority());
        }

        double noneFinalRemainingCurrent = remainingCurrent;
            
        for (Component component : components) {
            double additionalNeeded = component.getRequestedCurrent() - (component.getRequestedCurrent() * allocationRatio * 0.8);
            double additionalFairShare = (maxPriority-component.getPriority()) / totalPriority * noneFinalRemainingCurrent;
            double finalAdditionalAllocated = Math.min(additionalNeeded, additionalFairShare);
            component.allocateCurrent(component.getRequestedCurrent() * allocationRatio + finalAdditionalAllocated);
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
