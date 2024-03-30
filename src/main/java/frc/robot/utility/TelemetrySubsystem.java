package frc.robot.utility;

import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetrySubsystem implements Runnable {
    private static final ConcurrentHashMap<String, Object> telemetryData = new ConcurrentHashMap<>();

    public void start() {
        Thread thread = new Thread(this);
        thread.start();
    }

    private static void putValue(String key, Object value) {
        if (value instanceof BooleanSupplier) value = ((BooleanSupplier) value).getAsBoolean(); // Convert BooleanSupplier to boolean
        if (value instanceof DoubleSupplier) value = ((DoubleSupplier) value).getAsDouble(); // Convert DoubleSupplier to double

        Object oldValue = telemetryData.get(key);
        if (oldValue != null && oldValue.equals(value)) return; // Don't update if the value hasn't changed

        telemetryData.put(key, value); // Update the value or add it if it doesn't exist

        if (value instanceof Double) SmartDashboard.putNumber(key, (Double) value);
        else if (value instanceof Integer) SmartDashboard.putNumber(key, (Integer) value);
        else if (value instanceof String) SmartDashboard.putString(key, (String) value);
        else if (value instanceof Boolean) SmartDashboard.putBoolean(key, (Boolean) value);
        else if (value instanceof Sendable) SmartDashboard.putData(key, (Sendable) value);
        else System.err.println("Unknown type for telemetry value: " + value.getClass().getName());
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            TelemetryUpdater.forEachTelemetryValue(TelemetrySubsystem::putValue);

            try {
                Thread.sleep(100); // TODO: Maybe sync with the rio clock to make more consistent?
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
