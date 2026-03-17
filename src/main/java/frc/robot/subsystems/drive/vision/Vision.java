package frc.robot.subsystems.drive.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.vision.VisionCamera.MeasurementConsumer;
import frc.robot.subsystems.drive.vision.VisionConstants.CameraConstants;
import frc.robot.subsystems.drive.vision.VisionConstants.SimulationConstants;

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

        // Restored: The consumer that actually passes vision data to the drivetrain
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

        if (RobotBase.isSimulation()) {
            setupSim();
        }
    }

    private void initCameras(MeasurementConsumer consumer) {
        cameras.add(
            new PVCamera.PVCameraConfig(CameraConstants.LEFT_CAMERA_NAME)
                .withTransform(CameraConstants.ROBOT_TO_LEFT_CAM)
                .setEnabled(CameraConstants.ENABLE_LEFT_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );

        cameras.add(
            new PVCamera.PVCameraConfig(CameraConstants.RIGHT_CAMERA_NAME)
                .withTransform(CameraConstants.ROBOT_TO_RIGHT_CAM)
                .setEnabled(CameraConstants.ENABLE_RIGHT_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );

        cameras.add(
            new PVCamera.PVCameraConfig(CameraConstants.CENTER_CAMERA_NAME)
                .withTransform(CameraConstants.ROBOT_TO_CENTER_CAM)
                .setEnabled(CameraConstants.ENABLE_CENTER_CAMERA)
                .withMeasurementConsumer(consumer)
                .build()
        );
    }

    private void setupSim() {
        visionSim = new VisionSystemSim("main");

        visionSim.addAprilTags(VisionConstants.getAprilTagLayout());

        for (VisionCamera cam : cameras) {
            if (cam instanceof PVCamera pvCamera) {
                SimCameraProperties cameraProp = new SimCameraProperties();

                cameraProp.setCalibration(
                    SimulationConstants.SIM_CAMERA_WIDTH,
                    SimulationConstants.SIM_CAMERA_HEIGHT,
                    SimulationConstants.SIM_CAMERA_FOV
                );

                cameraProp.setCalibError(
                    SimulationConstants.SIM_CALIB_AVG_ERROR,
                    SimulationConstants.SIM_CALIB_STD_DEV
                );

                cameraProp.setFPS(SimulationConstants.SIM_FPS);
                cameraProp.setAvgLatencyMs(SimulationConstants.SIM_AVG_LATENCY_MS);
                cameraProp.setLatencyStdDevMs(SimulationConstants.SIM_LATENCY_STD_DEV_MS);

                PhotonCameraSim camSim = new PhotonCameraSim(
                    pvCamera.getCamera(),
                    cameraProp
                );
                camSim.enableDrawWireframe(SimulationConstants.SIM_ENABLE_WIREFRAME);
                visionSim.addCamera(camSim, cam.getRobotToCamera());
            }
        }
    }

    public Pose2d getEstimatedPose() {
        return drive.getState().Pose;
    }

    public SwerveDriveState getDriveState() {
        return drive.getState();
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
