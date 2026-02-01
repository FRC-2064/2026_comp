package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class LianaHelpers {
    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    private static NetworkTableEntry getEntry(String key) {
        return ntInstance.getEntry(key);
    }

    public static double getFlywheelAdjustment(){
        return (double) getEntry("/Liana/Flywheel/AngularVelocity").getDouble(0);
    }

    public static double getHoodAngleAdjustment(){
        return (double) getEntry("Liana/Hood/Angle").getDouble(0);
    }

    public static double getTurretAngleAdjustment(){
        return (double) getEntry("Liana/Turret/Angle").getDouble(0);
    }

    public static void currentActiveHub(String phase){
        getEntry("Liana/Hub/Phase").setString(phase);
    }
}