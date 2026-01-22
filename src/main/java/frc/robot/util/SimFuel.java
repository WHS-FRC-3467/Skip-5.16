// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.FieldConstants;
import lombok.Getter;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

public class SimFuel {

    @Getter
    private static final SimFuel instance = new SimFuel();

    private Pose2d robotPose;

    public int fuelStored = 8;

    public List<Pose3d> depotFuel = new ArrayList<>();
    public List<Pose3d> neutralZoneFuel = new ArrayList<>();

    public SimFuel() {
        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 4; col++) {
                double x = FieldConstants.DEPOT_FIRST_BALL.getX() + (col * FieldConstants.FUEL_DIAMETER.in(Meters));
                double y = FieldConstants.DEPOT_FIRST_BALL.getY() + (row * FieldConstants.FUEL_DIAMETER.in(Meters));
                depotFuel.add(new Pose3d(x, y, FieldConstants.FUEL_DIAMETER.in(Meters)/2, new Rotation3d()));
            }
        }

        for (int row = 0; row < 30; row++) {
            for (int col = 0; col < 12; col++) {
                double x = FieldConstants.NEUTRAL_ZONE_FIRST_FUEL.getX() + (col * FieldConstants.FUEL_DIAMETER.in(Meters));
                double y = FieldConstants.NEUTRAL_ZONE_FIRST_FUEL.getY() + (row * FieldConstants.FUEL_DIAMETER.in(Meters));
                neutralZoneFuel.add(new Pose3d(x, y, FieldConstants.FUEL_DIAMETER.in(Meters)/2, new Rotation3d()));
            }
        }
        Logger.recordOutput("Sim/Stored Fuel",  fuelStored);
        Logger.recordOutput("Sim/Depot Fuel", depotFuel.toArray(new Pose3d[0]));
        Logger.recordOutput("Sim/Neutral Zone Fuel", neutralZoneFuel.toArray(new Pose3d[0]));
    }

    public void checkCollisions(Supplier<Pose2d> robotPose) {
        List<Pose3d> toRemove = new ArrayList<>();
        for (Pose3d fuelPose : depotFuel) {
            double distance = robotPose.get().getTranslation().getDistance(fuelPose.toPose2d().getTranslation());
            if (distance < FieldConstants.FUEL_DIAMETER.in(Meters)) {
                toRemove.add(fuelPose);
                fuelStored++;
            }
        }
        depotFuel.removeAll(toRemove);

        List<Pose3d> neutralToRemove = new ArrayList<>();
        for (Pose3d fuelPose : neutralZoneFuel) {
            double distance = this.robotPose.getTranslation().getDistance(fuelPose.toPose2d().getTranslation());
            if (distance < FieldConstants.FUEL_DIAMETER.in(Meters)) {
                neutralToRemove.add(fuelPose);
                fuelStored++;
            }
        }
        neutralZoneFuel.removeAll(neutralToRemove);

        Logger.recordOutput("Sim/Stored Fuel",  fuelStored);
        Logger.recordOutput("Sim/Depot Fuel", depotFuel.toArray(new Pose3d[0]));
        Logger.recordOutput("Sim/Neutral Zone Fuel", neutralZoneFuel.toArray(new Pose3d[0]));
    }

    public void removeFuel() {
        if (fuelStored > 0) {
            fuelStored--;
        }
        Logger.recordOutput("Sim/Stored Fuel",  fuelStored);
    }
}
