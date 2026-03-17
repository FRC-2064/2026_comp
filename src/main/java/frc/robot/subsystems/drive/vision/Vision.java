package frc.robot.subsystems.drive.vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.VisionCamera.MeasurementConsumer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {

    private final List<VisionCamera> cameras;
    private final CommandSwerveDrivetrain drive;
    private VisionSystemSim visionSim;

    public Vision(CommandSwerveDrivetrain drive) {
        this.cameras = new ArrayList<>();
        this.drive = drive;

        MeasurementConsumer consumer = measurement -> {
            if (measurement.isValid()) {
                drive.addVisionMeasurement(
                    measurement.pose,
                    measurement.timestampSeconds,
                    measurement.stdDevs
                );
            }
        };

        initCameras(consumer);

        if (RobotBase.isSimulation()) setupSim();
    }

    private void initCameras(MeasurementConsumer consumer) {
        cameras.add(
            new PVCamera.PVCameraConfig(VisionConstants.LEFT_CAMERA_NAME)
                .withTransform(VisionConstants.ROBOT_TO_LEFT_CAM)
                .setEnabled(VisionConstants.ENABLE_LEFT_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );

        cameras.add(
            new PVCamera.PVCameraConfig(VisionConstants.RIGHT_CAMERA_NAME)
                .withTransform(VisionConstants.ROBOT_TO_RIGHT_CAM)
                .setEnabled(VisionConstants.ENABLE_RIGHT_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );

        cameras.add(
            new PVCamera.PVCameraConfig(VisionConstants.CENTER_CAMERA_NAME)
                .withTransform(VisionConstants.ROBOT_TO_CENTER_CAM)
                .setEnabled(VisionConstants.ENABLE_CENTER_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );

        // cameras.add(
        //     new LLCamera.LLCameraConfig(VisionConstants.LIMELIGHT_NAME)
        //         .withTransform(VisionConstants.ROBOT_TO_LIMELIGHT_CAM)
        //         .setEnabled(VisionConstants.ENABLE_LIMELIGHT_CAMERA)
        //         .withMegaTag2(VisionConstants.USE_MGT2)
        //         .withMeasurementConsumer(consumer)
        //         .build()
        // );
    }

    private void setupSim() {
        visionSim = new VisionSystemSim("main");

        visionSim.addAprilTags(VisionConstants.getAprilTagLayout());

        for (VisionCamera cam : cameras) {
            if (cam instanceof PVCamera pvCamera) {
                SimCameraProperties cameraProp = new SimCameraProperties();

                cameraProp.setCalibration(
                    VisionConstants.SIM_CAMERA_WIDTH,
                    VisionConstants.SIM_CAMERA_HEIGHT,
                    VisionConstants.SIM_CAMERA_FOV
                );

                cameraProp.setCalibError(
                    VisionConstants.SIM_CALIB_AVG_ERROR,
                    VisionConstants.SIM_CALIB_STD_DEV
                );

                cameraProp.setFPS(VisionConstants.SIM_FPS);
                cameraProp.setAvgLatencyMs(VisionConstants.SIM_AVG_LATENCY_MS);
                cameraProp.setLatencyStdDevMs(
                    VisionConstants.SIM_LATENCY_STD_DEV_MS
                );

                PhotonCameraSim camSim = new PhotonCameraSim(
                    pvCamera.getCamera(),
                    cameraProp
                );
                camSim.enableDrawWireframe(
                    VisionConstants.SIM_ENABLE_WIREFRAME
                );
                visionSim.addCamera(camSim, pvCamera.getPosition());
            }
        }
    }

    @Override
    public void periodic() {
        for (VisionCamera cam : cameras) {
            cam.periodic();
        }
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) {
            visionSim.update(drive.getState().Pose);
        }
    }

    public int getEnabledCameraCount() {
        return (int) cameras.stream().filter(VisionCamera::isEnabled).count();
    }

    public Optional<VisionCamera> getCamera(String name) {
        return cameras
            .stream()
            .filter(cam -> cam.getName().equals(name))
            .findFirst();
    }

    public void setCameraEnabled(String name, boolean enabled) {
        getCamera(name).ifPresent(cam -> cam.setEnabled(enabled));
    }
}
