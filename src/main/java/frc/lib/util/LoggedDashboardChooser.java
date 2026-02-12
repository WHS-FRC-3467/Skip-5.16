// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/util/LoggedDashboardChooser.java

package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/**
 * A dashboard chooser that integrates with AdvantageKit's logging system.
 *
 * <p>
 * This class wraps WPILib's SendableChooser to provide automatic logging of selected values. Unlike
 * the standard SendableChooser, this version works correctly with AdvantageKit's replay
 * functionality, allowing you to replay autonomous selections and other dashboard choices from log
 * files.
 *
 * <p>
 * Example usage:
 *
 * <pre>{@code
 * LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Mode");
 * autoChooser.addDefaultOption("Do Nothing", Commands.none());
 * autoChooser.addOption("Simple Auto", simpleAuto());
 * autoChooser.addOption("Complex Auto", complexAuto());
 *
 * // Get selected command
 * Command selectedAuto = autoChooser.get();
 * }</pre>
 */
public class LoggedDashboardChooser<V> extends LoggedNetworkInput {
    private final String key;
    private String selectedValue = null;
    private String lastSelected = null;
    private SendableChooser<String> sendableChooser = new SendableChooser<>();
    private Map<String, V> options = new HashMap<>();
    private Consumer<V> listener = null;

    private final LoggableInputs inputs =
        new LoggableInputs() {
            @Override
            public void toLog(LogTable table) {
                table.put(key, selectedValue);
            }

            @Override
            public void fromLog(LogTable table) {
                selectedValue = table.get(key, selectedValue);
            }
        };

    /**
     * Creates a new LoggedDashboardChooser.
     *
     * @param key The SmartDashboard key, published to "/SmartDashboard/{key}" for NT or
     *        "/DashboardInputs/{key}" when logged
     */
    public LoggedDashboardChooser(String key) {
        this.key = key;
        SmartDashboard.putData(key, sendableChooser);
        periodic();
        Logger.registerDashboardInput(this);
    }

    /**
     * Creates a new LoggedDashboardChooser by copying options from an existing SendableChooser.
     * Note: Updates to the original chooser after construction will not affect this object.
     *
     * @param key The SmartDashboard key for this chooser
     * @param chooser Existing SendableChooser to copy options from
     */
    @SuppressWarnings("unchecked")
    public LoggedDashboardChooser(String key, SendableChooser<V> chooser) {
        this(key);

        // Get options map
        Map<String, V> options = new HashMap<>();
        try {
            Field mapField = SendableChooser.class.getDeclaredField("m_map");
            mapField.setAccessible(true);
            options = (Map<String, V>) mapField.get(chooser);
        } catch (NoSuchFieldException
            | SecurityException
            | IllegalArgumentException
            | IllegalAccessException e) {
            throw new IllegalStateException(e.getMessage());
        }

        // Get default option
        String defaultString = "";
        try {
            Field defaultField = SendableChooser.class.getDeclaredField("m_defaultChoice");
            defaultField.setAccessible(true);
            defaultString = (String) defaultField.get(chooser);
        } catch (NoSuchFieldException
            | SecurityException
            | IllegalArgumentException
            | IllegalAccessException e) {
            throw new IllegalStateException(e.getMessage());
        }

        // Add options
        for (String optionKey : options.keySet()) {
            if (optionKey.equals(defaultString)) {
                addDefaultOption(optionKey, options.get(optionKey));
            } else {
                addOption(optionKey, options.get(optionKey));
            }
        }
    }

    /**
     * Adds a new option to the chooser.
     *
     * @param key Display name for the option
     * @param value Value returned when this option is selected
     */
    public void addOption(String key, V value) {
        sendableChooser.addOption(key, key);
        options.put(key, value);
    }

    /**
     * Adds a new option and sets it as the default.
     *
     * @param key Display name for the default option
     * @param value Value returned when this option is selected
     */
    public void addDefaultOption(String key, V value) {
        sendableChooser.setDefaultOption(key, key);
        options.put(key, value);
    }

    /**
     * Returns the currently selected option value. If no option is selected, returns the default.
     * If no default exists, returns null.
     *
     * @return The selected value, or null if nothing is selected
     */
    public V get() {
        return options.get(selectedValue);
    }

    /**
     * Returns the internal SendableChooser for dashboard layout configuration. Do not read data
     * from this directly - use {@link #get()} instead.
     *
     * @return The internal SendableChooser object
     */
    public SendableChooser<String> getSendableChooser() {
        return sendableChooser;
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            selectedValue = sendableChooser.getSelected();
        }
        Logger.processInputs(prefix, inputs);

        if (listener != null && !selectedValue.equals(lastSelected)) {
            listener.accept(get());
        }
        lastSelected = selectedValue;
    }

    /**
     * Registers a listener to be called when the selected option changes.
     *
     * @param listener Consumer to be called with the new value when selection changes
     */
    public void onChange(Consumer<V> listener) {
        this.listener = listener;
    }
}
