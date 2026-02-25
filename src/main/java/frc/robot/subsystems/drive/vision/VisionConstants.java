package frc.robot.subsystems.drive.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {

    public static final String LEFT_CAMERA_NAME = "leftCam";
    public static final String RIGHT_CAMERA_NAME = "rightCam";
    public static final String CENTER_CAMERA_NAME = "centerCam";
    public static final String LIMELIGHT_NAME = "limelight-one";

    public static final Transform3d ROBOT_TO_LEFT_CAM = new Transform3d(
        new Translation3d(
            Inches.of(-13.592491),
            Inches.of(0),
            Inches.of(12.045690)
        ),
        new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(0))
    );

    public static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(
        new Translation3d(Inches.of(-10.500000), Inches.of(-11.500000), Inches.of(8.000000)),
        new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-60))
    );

    public static final Transform3d ROBOT_TO_CENTER_CAM = new Transform3d(
        new Translation3d(Inches.of(-10.500000), Inches.of(11.500000), Inches.of(8.000000)),
        new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(30))
    );

    public static final Transform3d ROBOT_TO_LIMELIGHT_CAM = new Transform3d(
        new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(
        4.0,
        4.0,
        8.0
    );

    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(
        0.5,
        0.5,
        1.0
    );

    public static final boolean USE_MGT2 = true;

    public static final double MAX_POSE_AMBIGUITY = 0.2;

    public static final double MAX_DETECTION_DISTANCE = 6.0;

    public static final int MIN_TAG_COUNT = 1;

    public static final boolean USE_MULTI_TAG_FALLBACK = true;

    public static final boolean ENABLE_LEFT_CAMERA = true;
    public static final boolean ENABLE_RIGHT_CAMERA = true;
    public static final boolean ENABLE_CENTER_CAMERA = true;
    public static final boolean ENABLE_LIMELIGHT_CAMERA = true;

    public static final int SIM_CAMERA_WIDTH = 960;
    public static final int SIM_CAMERA_HEIGHT = 720;

    public static final Rotation2d SIM_CAMERA_FOV = Rotation2d.fromDegrees(90);

    public static final double SIM_CALIB_AVG_ERROR = 0.35;
    public static final double SIM_CALIB_STD_DEV = 0.10;

    public static final int SIM_FPS = 60;
    public static final double SIM_AVG_LATENCY_MS = 35;
    public static final double SIM_LATENCY_STD_DEV_MS = 5;

    public static final boolean SIM_ENABLE_WIREFRAME = true;

    public static AprilTagFieldLayout getAprilTagLayout() {
        return frc.robot.utils.FieldConstants.defaultAprilTagType.getLayout();
    }
}
