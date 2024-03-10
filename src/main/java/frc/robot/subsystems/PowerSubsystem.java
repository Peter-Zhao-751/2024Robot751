package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

public class PowerSubsystem extends SubsystemBase {
    private final PowerDistribution powerDistribution;

    /**
     * Creates a new PowerSubsystem which is a wrapper for the PowerDistribution class
     * @see PowerDistribution
     * @see PowerDistribution.ModuleType
     */
    public PowerSubsystem() {
        powerDistribution = new PowerDistribution();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        TelemetryUpdater.setTelemetryValue("PDH Voltage", powerDistribution.getVoltage());
        TelemetryUpdater.setTelemetryValue("PDH Temperature", powerDistribution.getTemperature());
        TelemetryUpdater.setTelemetryValue("PDH Total Current", powerDistribution.getTotalCurrent());
        //TelemetryUpdater.setTelemetryValue("PDH Total Power", powerDistribution.getTotalPower());
        //TelemetryUpdater.setTelemetryValue("PDH Total Energy", powerDistribution.getTotalEnergy());
        //TelemetryUpdater.setTelemetryValue("PDH Total Channel", powerDistribution.getNumChannels());
    }
}
