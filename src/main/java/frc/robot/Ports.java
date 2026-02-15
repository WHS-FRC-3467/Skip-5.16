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

package frc.robot;

import com.ctre.phoenix6.CANBus;
import frc.lib.util.Device;
import frc.lib.util.Device.CAN;

/**
 * Hardware port definitions for all CAN devices and other I/O ports on the robot. Contains CAN IDs
 * for motor controllers, sensors, and other devices connected to the robot.
 */
public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    // Don't use the rio for drivetrain:
    // - It has slower odometry
    // - We assign CAN ID 0 to the pigeon
    public static final CANBus DRIVETRAIN_BUS = new CANBus("Drivetrain");

    public static final Device.CAN lights = new CAN(7, "Drivetrain");

    public static final Device.CAN pdh = new CAN(40, "rio");

    public static final Device.CAN leftFlywheelMain = new CAN(18, "rio");
    public static final Device.CAN leftFlywheelFollower = new CAN(16, "rio");
    public static final Device.CAN rightFlywheelMain = new CAN(21, "rio");
    public static final Device.CAN rightFlywheelFollower = new CAN(22, "rio");

    public static final Device.CAN hood = new CAN(19, "rio");

    public static final Device.CAN tower = new CAN(20, "rio");
    public static final Device.CAN towerLaserCAN1 = new CAN(17, "rio");
    public static final Device.CAN towerLaserCAN2 = new CAN(23, "rio");

    public static final Device.CAN indexerFloor = new CAN(25, "rio");
    // TODO: Ensure ID is correct
    // public static final Device.CAN indexerFloorFollower = new CAN(26, "rio");
    public static final Device.CAN indexerCentering = new CAN(24, "rio");

    public static final Device.CAN intakeLinear = new CAN(27, "rio");
    public static final Device.CAN intakeRoller = new CAN(28, "rio");

    public static final Device.CAN climber = new CAN(30, "rio"); // placeholder
}
