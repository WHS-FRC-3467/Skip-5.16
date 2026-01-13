package frc.lib.devices;

import frc.lib.io.objectdetection.ObjectDetectionIO;
import frc.lib.io.objectdetection.ObjectDetectionIO.ObjectDetectionIOInputs;

/**
 * Device level implementation of an Color Detection camera. While the IO layer is responsible for
 * defining the variables of interest coming from our camera, the device layer is responsible for
 * periodically polling that IO and performing relevant calculations on the return results to
 * generate data for the robot to make decisions. Note that this device shares its real IO layer 
 * with ObjectDetection. 
 */
public class ColorDetection {
    // IO implementation of ObjectDetectionIO
    private final ObjectDetectionIO io;
    private final ObjectDetectionIOInputs inputs = new ObjectDetectionIOInputs();

    /*
     * Interface as a data type allows ColorDetection to accept various implementations of
     * ObjectDetectionIO (e.g. ObjectDetectionIOPhotonVision or ObjectDetectionIOLimelight).
     * Currently factored for PhotonVision only.
     */
    public ColorDetection(ObjectDetectionIO io) {
        this.io = io;
    }

    /*
     * Periodically retrive most recent Color pipeline results and populate into inputs.
     */
    public void periodic() {
        io.updateInputs(inputs);
    }
}
