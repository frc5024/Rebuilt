package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FuelCellConstants;

/**
 * 
 */
public class FuelSimCount {
    private final double launchBPS = 8.0;

    private int fuelStored;

    public FuelSimCount(int fuelStored) {
        this.fuelStored = fuelStored;
    }

    /**
     * 
     */
    public Pose3d[] getFuelInRobotPoses(Pose2d robotPose) {
        ArrayList<Pose3d> poses = new ArrayList<>();

        int index = 0;
        int size = 3;
        double pX = 0.0, pY = 0.0, pZ = 0.0;
        for (int z = 0; z < size; z++) {
            for (int y = 0; y < size; y++) {
                for (int x = 0; x < size; x++) {
                    if (index < getFuelStored()) {
                        pX = (-FuelCellConstants.DIAMETER / 2) + (FuelCellConstants.DIAMETER * x);
                        pY = -FuelCellConstants.DIAMETER + (FuelCellConstants.DIAMETER * y);
                        pZ = (FuelCellConstants.DIAMETER * z) + FuelCellConstants.DIAMETER;

                        Transform3d transform3d = new Transform3d(pX, pY, pZ, new Rotation3d());
                        Pose3d pose = new Pose3d(robotPose).transformBy(transform3d);
                        poses.add(pose);
                        index++;
                    } else {
                        return poses.toArray(new Pose3d[0]);
                    }
                }
            }
        }

        return poses.toArray(new Pose3d[0]);
    }

    public int getFuelStored() {
        return this.fuelStored;
    }

    public double getLaunchBPS() {
        return this.launchBPS;
    }

    public void setFuelStored(int fuelStored) {
        this.fuelStored = fuelStored;
    }
}
