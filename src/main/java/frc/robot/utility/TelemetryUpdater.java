package frc.robot.utility;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BiConsumer;

public class TelemetryUpdater {
    private static final ConcurrentHashMap<String, Object> telemetryData = new ConcurrentHashMap<>();

    // Private constructor to prevent instantiation
    private TelemetryUpdater() {}

    public static void setTelemetryValue(String key, Object value) {
        telemetryData.put(key, value);
    }

    public static Object getTelemetryValue(String key) {
        return telemetryData.get(key);
    }
    
    public static void forEachTelemetryValue(BiConsumer<String, Object> action) {
        telemetryData.forEach(action);
    }
}
