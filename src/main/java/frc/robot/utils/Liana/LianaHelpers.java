package frc.robot.utils.Liana;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LianaHelpers {
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private static final DoubleSubscriber flywheelSub = nt.getDoubleTopic("/Liana/Flywheel/AngularVelocity").subscribe(0.0);
    private static final DoubleSubscriber hoodSub = nt.getDoubleTopic("/Liana/Hood/Angle").subscribe(0.0);
    private static final DoubleSubscriber turretSub = nt.getDoubleTopic("/Liana/Turret/Angle").subscribe(0.0);

    public static double getFlywheelAdjustment() {
        return flywheelSub.get();
    }

    public static double getHoodAngleAdjustment() {
        return hoodSub.get();
    }

    public static double getTurretAngleAdjustment() {
        return turretSub.get();
    }
}
