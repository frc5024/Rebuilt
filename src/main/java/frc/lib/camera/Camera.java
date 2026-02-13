package frc.lib.camera;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * 
 */
public class Camera {
    private final String name;
    private final String ntName;
    private final Transform3d cameraToRobot;

    /**
     *  Robot to camera transforms are NOT used by Limelight, configure in web UI instead
     */
    public Camera(String name, String ntName, Transform3d cameraToRobot) {
        this.name = name;
        this.ntName = ntName;
        this.cameraToRobot = cameraToRobot;
    }

    /**
     * 
     */
    public String getName() {
        return this.name;
    }

    /**
     * 
     */
    public String getNtName() {
        return this.ntName;
    }

    /**
     * 
     */
    public Transform3d getCameraToRobot() {
        return this.cameraToRobot;
    }
}
