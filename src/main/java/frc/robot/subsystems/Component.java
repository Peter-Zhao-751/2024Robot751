package frc.robot.subsystems;

public interface Component {
    double getCurrentDraw();
    void allocateCurrent(double current);
    int getPriority();
}
