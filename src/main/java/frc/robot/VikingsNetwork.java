package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VikingsNetwork {

    public static NetworkTable table() {
        return NetworkTableInstance.getDefault().getTable("Vikings");
    }

}
