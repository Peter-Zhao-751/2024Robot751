package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TelemetryUpdater;

public class PowerSubsystem extends SubsystemBase {
    private static PowerSubsystem instance;
    private final PowerDistribution powerDistribution;

    public static PowerSubsystem getInstance() {
        if (instance == null) instance = new PowerSubsystem();
        return instance;
    }

    /**
     * Creates a new PowerSubsystem which is a wrapper for the PowerDistribution class
     * @see PowerDistribution
     * @see PowerDistribution.ModuleType
     */
    private PowerSubsystem() {
        powerDistribution = new PowerDistribution(Constants.Swerve.pdhId, PowerDistribution.ModuleType.kRev);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Voltage", powerDistribution.getVoltage());
        //TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Temperature", powerDistribution.getTemperature());
        TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Total Current", powerDistribution.getTotalCurrent());
        //TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Total Power", powerDistribution.getTotalPower());
        //TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Total Energy", powerDistribution.getTotalEnergy());
        TelemetryUpdater.setTelemetryValue("PowerSubsystem/PDH Total Channel", powerDistribution.getNumChannels());
    }
}
