package frc.robot.subsystems;

public interface Component {

    /**
     * Get the current draw of the component
     * used to allocate current
     * @return double the current draw of the component
     */
    double getCurrentDraw();

    /**
     * Allocate current to the component
     * @param current the current to allocate
     */
    void allocateCurrent(double current);
    
    /**
     * Get the priority of the component
     * used to allocate current
     * @return int the priority of the component
     */
    int getPriority();
}
