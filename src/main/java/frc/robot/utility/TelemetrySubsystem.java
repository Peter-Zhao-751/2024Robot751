package frc.robot.utility;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetrySubsystem implements Runnable {

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            TelemetryUpdater.forEachTelemetryValue((key, value) -> {
                if (value instanceof Double) {
                    SmartDashboard.putNumber(key, (Double) value);
                } else if (value instanceof String) {
                    SmartDashboard.putString(key, (String) value);
                } else if (value instanceof Boolean) {
                    SmartDashboard.putBoolean(key, (Boolean) value);
                } else if (value instanceof Sendable){
                    SmartDashboard.putData(key, (Sendable) value);
                }
                // Add handling for other types as needed
            });

            try {
                Thread.sleep(100); // Sleep to limit update frequency
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Properly handle interruption
            }
        }
    }

    // Start the thread from within the class if desired
    public void start() {
        Thread thread = new Thread(this);
        thread.start();
    }
}
