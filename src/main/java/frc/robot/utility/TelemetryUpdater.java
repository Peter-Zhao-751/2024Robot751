package frc.robot.utility;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BiConsumer;

public class TelemetryUpdater {
    private static TelemetryUpdater instance;
    private final ConcurrentHashMap<String, Object> telemetryData = new ConcurrentHashMap<>();

    // Private constructor to prevent instantiation
    private TelemetryUpdater() {}

    // Public method to get the instance
    public static synchronized TelemetryUpdater getInstance() {
        if (instance == null) {
            instance = new TelemetryUpdater();
        }
        return instance;
    }

    public void setTelemetryValue(String key, Object value) {
        telemetryData.put(key, value);
    }

    public Object getTelemetryValue(String key) {
        return telemetryData.get(key);
    }
    
    public void forEachTelemetryValue(BiConsumer<String, Object> action) {
        telemetryData.forEach(action);
    }
}
