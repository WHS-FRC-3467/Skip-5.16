// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;

/** Util for tracking pose of a simulated gamepiece, used for visualization in AScope */
public class GamePieceVisualizer {

    private final String name;

    @Getter
    Pose3d gamePiecePose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    /**
     * Instance of virtual gamepiece
     * 
     * @param name Name of instance
     * @param pose3d Pose3d of instance
     */
    public GamePieceVisualizer(String name, Pose3d pose3d)
    {
        this.name = name;
        this.gamePiecePose = pose3d;
        Logger.recordOutput(name + " Visualizer", this.gamePiecePose);

    }

    public void setPose(Pose3d pose)
    {
        this.gamePiecePose = pose;
        Logger.recordOutput(name + " Visualizer", this.gamePiecePose);
    }

    public void hide()
    {
        this.gamePiecePose = new Pose3d();
        Logger.recordOutput(name + " Visualizer", this.gamePiecePose);
    }
}
