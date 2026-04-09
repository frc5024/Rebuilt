package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.camera.Camera;
import frc.robot.Constants.VisionConstants;

/** 
 * 
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name         The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(Camera camera, Supplier<Pose2d> poseSupplier) {
        super(camera);

        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        this.cameraSim = new PhotonCameraSim(this.photonCamera, cameraProperties, VisionConstants.aprilTagLayout);
        visionSim.addCamera(this.cameraSim, this.robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(this.poseSupplier.get());
        super.updateInputs(inputs);
    }
}
