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

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.UUID;

class VisionIOCamCamTest {

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void updateInputsIgnoresDefaultSubscriberValue() {
        VisionIOCamCam io = new VisionIOCamCam("vision-io-test-" + UUID.randomUUID());
        VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

        try {
            io.updateInputs(inputs);

            assertEquals(0, inputs.unreadData.length);
        } finally {
            io.close();
        }
    }

    @Test
    void updateInputsPreservesAllUnreadQueueValues() {
        String instanceName = "vision-io-test-" + UUID.randomUUID();
        VisionIOCamCam io = new VisionIOCamCam(instanceName);
        RawPublisher publisher =
                NetworkTableInstance.getDefault()
                        .getTable("CamCam")
                        .getSubTable(instanceName)
                        .getRawTopic("Data")
                        .publish("CamCamData");
        VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

        try {
            publisher.set(new byte[] {1});
            publisher.set(new byte[] {2});

            io.updateInputs(inputs);

            assertEquals(2, inputs.unreadData.length);
            assertArrayEquals(new byte[] {1}, inputs.unreadData[0]);
            assertArrayEquals(new byte[] {2}, inputs.unreadData[1]);

            io.updateInputs(inputs);

            assertEquals(0, inputs.unreadData.length);
        } finally {
            publisher.close();
            io.close();
        }
    }

    @Test
    void configurePublishesRawConfiguration() {
        String instanceName = "vision-io-test-" + UUID.randomUUID();
        VisionIOCamCam io = new VisionIOCamCam(instanceName);
        RawSubscriber subscriber =
                NetworkTableInstance.getDefault()
                        .getTable("CamCam")
                        .getSubTable(instanceName)
                        .getRawTopic("Config")
                        .subscribe(
                                "CamCamConfiguration",
                                new byte[] {},
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(4));

        try {
            io.configure(new byte[] {9, 8, 7});

            TimestampedRaw[] queue = subscriber.readQueue();
            assertEquals(1, queue.length);
            assertArrayEquals(new byte[] {9, 8, 7}, queue[0].value);
        } finally {
            subscriber.close();
            io.close();
        }
    }
}
