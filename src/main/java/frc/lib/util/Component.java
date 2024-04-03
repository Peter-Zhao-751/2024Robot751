package frc.lib.util;

public interface Component {
    /**
     * Get the priority of the component
     * @return the priority of the component
     */
    int getPriority();

    /**
     * Get the current draw of the component
     * @return the current draw of the component
     */
    double getCurrentDraw();

    /**
     * Allocate current to the component
     * @param current the current to allocate
     */
    void allocateCurrent(double current);
}
