package frc.robot.utility;
import java.util.ArrayList;
import java.util.Comparator;

import frc.lib.util.Component;
import frc.robot.Constants;

@Deprecated
public class CurrentManager {
    private static CurrentManager instance;

    public final double maxCurrent;
    public final double maxPercent;
    public final double nominalPercent;
    public double totalCurrent;

    private final ArrayList<Component> components = new ArrayList<>();

    public static CurrentManager getInstance() {
        if (instance == null) instance = new CurrentManager();
        return instance;
    }

    private CurrentManager() {
        maxCurrent = Constants.CurrentManager.maxCurrent;
        maxPercent = Constants.CurrentManager.maxPercent;
        nominalPercent = Constants.CurrentManager.nominalPercent;
        totalCurrent = 0;
    }

    public void addComponent(Component component) {
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

    public double getCurrent(){
        return totalCurrent;
    }
    public double getPercent(){
        return totalCurrent / maxCurrent;
    }
    public boolean isOverNominal(){
        return getPercent() > nominalPercent;
    }
    public boolean isOverMax(){
        return getPercent() > maxPercent;
    }
}
