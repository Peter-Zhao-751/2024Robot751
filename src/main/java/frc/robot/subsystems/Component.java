package frc.robot.subsystems;

public interface Component {
    double getRequestedCurrent();
    void allocateCurrent(double current);
    int getPriority();
    void updateBasedOnAllocatedCurrent();
}
