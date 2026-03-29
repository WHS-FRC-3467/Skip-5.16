/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.io.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;

import frc.robot.generated.flatbuffers.UnconfiguredError;

/**
 * Real hardware implementation of VisionIO for a CamCam coprocessor.
 *
 * <p>Publishes configuration and reads unread pose batches over NetworkTables for real robot
 * operation.
 */
public class VisionIOCamCam implements VisionIO {
    private static final byte[] EMPTY_DATA = new byte[] {};
    private static final int DATA_QUEUE_DEPTH = 32;

    private final NetworkTable rootTable = NetworkTableInstance.getDefault().getTable("CamCam");

    private final NetworkTable table;

    /**
     * Rio -> Coprocessor. Sends configuration data to the coproc. If configuration fails, {@code
     * dataSubscriber} will continue to return {@link UnconfiguredError}
     */
    private final RawPublisher configurationPublisher;

    /**
     * Rio <- Coprocessor. Receives unread data updates from the coprocessor via NetworkTables'
     * built-in subscriber queue.
     */
    private final RawSubscriber dataSubscriber;

    /**
     * Creates a NetworkTables-backed transport for one CamCam instance.
     *
     * @param instanceName name of the CamCam NetworkTables subtable
     */
    public VisionIOCamCam(String instanceName) {
        table = rootTable.getSubTable(instanceName);
        configurationPublisher = table.getRawTopic("Config").publish("CamCamConfiguration");
        dataSubscriber =
                table.getRawTopic("Data")
                        .subscribe(
                                "CamCamData",
                                EMPTY_DATA,
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(DATA_QUEUE_DEPTH));
    }

    @Override
    public void configure(byte[] configuration) {
        configurationPublisher.accept(configuration);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        TimestampedRaw[] dataQueue = dataSubscriber.readQueue();
        if (dataQueue.length == 0) {
            inputs.unreadData = new byte[][] {};
            return;
        }

        int validEntries = 0;
        for (TimestampedRaw entry : dataQueue) {
            byte[] data = entry.value;
            if (data != null && data.length > 0) {
                validEntries++;
            }
        }

        if (validEntries == 0) {
            inputs.unreadData = new byte[][] {};
            return;
        }

        byte[][] unreadData = new byte[validEntries][];
        int nextIndex = 0;
        for (TimestampedRaw entry : dataQueue) {
            byte[] data = entry.value;
            if (data == null || data.length == 0) {
                continue;
            }

            unreadData[nextIndex++] = data;
        }

        inputs.unreadData = unreadData;
    }

    @Override
    public void close() {
        configurationPublisher.close();
        dataSubscriber.close();
    }
}
