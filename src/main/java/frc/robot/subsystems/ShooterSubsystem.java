package frc.robot.subsystems;
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        //CurrentManager.updateCurrent(1, CurrentManager.Subsystem.Shooter);
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
