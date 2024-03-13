package frc.robot.utility;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Path;
import java.util.Base64;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import edu.wpi.first.cscore.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants;

public class UIManager {
    private static final SendableChooser<File> autonSelector = new SendableChooser<>();
    private static GenericEntry updatePreferencesButton;
    private static GenericEntry resetPreferencesButton;
    private static File selectedAuton = null;
    private static CvSource imageSource;

    // updating ui methods
    public static void updatePathPreview() {
        File currentSelection = autonSelector.getSelected();
        if (currentSelection != null && !currentSelection.equals(selectedAuton)) {
            selectedAuton = currentSelection;
            String base64Image = Barn2PathInterpreter.getAutonPreview(selectedAuton);
            System.out.println("\n\nupdated auton path\n\n");
            byte[] base64ImageByte = Base64.getDecoder().decode(base64Image);
            Mat image = Imgcodecs.imdecode(new MatOfByte(base64ImageByte), Imgcodecs.IMREAD_UNCHANGED);
            imageSource.putFrame(image);
        }
    }

    /**
     * This function will go through the provided class (the Constants class) and
     * update each
     * preference based on the value of the constant. If the preference does not
     * exist, it will be
     * created. If it does exist, the Constant will be updated to match the
     * preference.
     */
    public static void updatePreferencesBasedOnConstants(Class<?> constants, boolean reset) {
        Field[] fields = constants.getDeclaredFields();
        for (Field field : fields) {
            if (Modifier.isPublic(field.getModifiers()) && Modifier.isStatic(field.getModifiers())) {
                if (reset) resetPreferenceToConstant(field);
                else updateOrRetrievePreference(field);
            }
        }

        // Recursively handle inner classes
        Class<?>[] innerClasses = constants.getDeclaredClasses();
        for (Class<?> innerClass : innerClasses) updatePreferencesBasedOnConstants(innerClass, reset);
    }

    /**
     * This function will update the preference based on the value of the field, or
     * retrieve the
     * preference and update the field's value based on the preference.
     */
    public static void updateOrRetrievePreference(Field field) {
        try {
            // Ensure the field is accessible
            field.setAccessible(true);
            // Check if a Preference exists for this field
            if (!Preferences.containsKey(field.getName())) {
                setPreferences(field);
            } else {
                // Preference exists, update field's value from Preference
                switch (field.getType().getName()) {
                    case "int" -> field.setInt(null, Preferences.getInt(field.getName(), field.getInt(null)));
                    case "double" ->
                            field.setDouble(null, Preferences.getDouble(field.getName(), field.getDouble(null)));
                    case "boolean" ->
                            field.setBoolean(null, Preferences.getBoolean(field.getName(), field.getBoolean(null)));
                    case "java.lang.String" ->
                            field.set(null, Preferences.getString(field.getName(), (String) field.get(null)));
                }
            }
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    private static void setPreferences(Field field) throws IllegalAccessException {
        switch (field.getType().getName()) {
            case "int" -> Preferences.setInt(field.getName(), field.getInt(null));
            case "double" -> Preferences.setDouble(field.getName(), field.getDouble(null));
            case "boolean" -> Preferences.setBoolean(field.getName(), field.getBoolean(null));
            case "java.lang.String" -> Preferences.setString(field.getName(), (String) field.get(null));
        }
    }

    /**
     * This function will reset the preference based on the value of the field, or
     * retrieve the
     * preference and reset the field's value based on the preference.
     */
    public static void resetPreferenceToConstant(Field field) {
        try {
            // Ensure the field is accessible
            field.setAccessible(true);
            // Check if a Preference exists for this field
            if (Preferences.containsKey(field.getName())) {
                // Preference exists, update preferences' value from field
                setPreferences(field);
            }
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public static void updateTelemetry() {

        // if (webcamCVSink.grabFrame(webcamSource) == 0) {
        //     // Send error message to dashboard to ensure the stream works even if there's an error
        //     webcamOutputStream.notifyError(webcamCVSink.getError());
        //     return;
        // }

        // Rotate the image frame
        //Core.rotate(webcamSource, webcamOutput, Core.ROTATE_180);

        // Send the processed frame to the Dashboard
        //webcamOutputStream.putFrame(webcamOutput);

        // TelemetryUpdater.setTelemetryValue("Current Manager Over Nominal", CurrentManager.isOverNominal());
        // TelemetryUpdater.setTelemetryValue("Current Manager Over Peak", CurrentManager.isOverMax());

        // if (updatePreferencesButton.getBoolean(false)) {
        //     updatePreferencesBasedOnConstants(Constants.class, false);
        //     updatePreferencesButton.setBoolean(false); // Reset the toggle button
        // }
        // if (resetPreferencesButton.getBoolean(false)) {
        //     updatePreferencesBasedOnConstants(Constants.class, true);
        //     resetPreferencesButton.setBoolean(false); // Reset the toggle button
        // }
    }

    public static void initializeUI() {
        // initializing webcam

        TelemetryUpdater.setTelemetryValue("Intake State", "waiting!!");

        new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);
            camera.setResolution(640, 480);
    
            CvSink cvSink = CameraServer.getVideo(camera);

            CvSource outputStream = CameraServer.putVideo("Front Fisheye", 640, 480);
        
            Mat mat = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) continue;
                Core.flip(mat, mat, 1);
        
                outputStream.putFrame(mat);
            }
        }).start();

        // initializing path stream
        imageSource = CameraServer.putVideo("Path Preview", 827, 401);
        CameraServer.startAutomaticCapture(imageSource);

        // sets a bunch of UI stuff
        Path deployDirectory = Filesystem.getDeployDirectory().toPath();
        Path barn2PathDirectory = deployDirectory.resolve("barn2path");

        File barn2PathDir = barn2PathDirectory.toFile();
        File[] filesList = barn2PathDir.listFiles();

        autonSelector.setDefaultOption("Simple Auton", null);
        for (File file : filesList) {
            autonSelector.addOption(file.getName(), file);
        }

        Shuffleboard.getTab("Auton Selector")
                .add("Select a Path:", autonSelector)
                .withWidget("Combo Box Chooser");

        updatePreferencesButton = Shuffleboard.getTab("Preferences")
                .add("Update Preferences", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();

        resetPreferencesButton = Shuffleboard.getTab("Preferences")
                .add("Reset Preferences", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
    }

    public static File selectedAuton() {
        File selectedAuton = autonSelector.getSelected();
        // TelemetryUpdater.setTelemetryValue("Current Action", "Autonomous: " + selectedAuton.getName());
        return selectedAuton;
    }
}
