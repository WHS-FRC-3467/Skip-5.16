// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.FieldConstants;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

public class SimFuel {

    public LoggedTunableNumber fuelStored = new LoggedTunableNumber("Sim/Fuel Stored", 8);

    public List<Pose3d> depotFuel = new ArrayList<>();

    public SimFuel() {
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 4; col++) {
                double x = FieldConstants.DEPOT_FIRST_BALL.getX() + (col * FieldConstants.FUEL_DIAMETER.in(Meters));
                double y = FieldConstants.DEPOT_FIRST_BALL.getY() + (row * FieldConstants.FUEL_DIAMETER.in(Meters));
                depotFuel.add(new Pose3d(x, y, FieldConstants.FUEL_DIAMETER.in(Meters)/2, new Rotation3d()));
            }
        }
        Logger.recordOutput("Sim/Depot Fuel", depotFuel.toArray(new Pose3d[0]));
    }
}
