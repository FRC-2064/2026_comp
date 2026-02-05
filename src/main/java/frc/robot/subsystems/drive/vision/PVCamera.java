package frc.robot.subsystems.drive.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.FieldConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PVCamera implements VisionCamera {

    private final String name;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final MeasurementConsumer consumer;
    private final Transform3d cameraPostion;
    private boolean enabled;
    private Matrix<N3, N1> curStdDevs;

    private PVCamera(PVCameraConfig config) {
        this.name = config.name;
        this.camera = new PhotonCamera(config.name);
        this.enabled = config.enabled;
        this.consumer = config.measurementConsumer;
        this.curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;

        this.cameraPostion = config.robotToCamera;

        this.poseEstimator = new PhotonPoseEstimator(
            FieldConstants.defaultAprilTagType.getLayout(),
            config.robotToCamera
        );
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        for (var result : camera.getAllUnreadResults()) {
            processResult(result);
        }
    }

    private void processResult(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

        if (visionEst.isEmpty() && VisionConstants.USE_MULTI_TAG_FALLBACK) {
            visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        updateEstimationStdDevs(visionEst, result.getTargets());

        visionEst.ifPresent(est -> {
            if (
                result.hasTargets() &&
                result.getBestTarget().getPoseAmbiguity() >
                VisionConstants.MAX_POSE_AMBIGUITY
            ) {
                return;
            }

            VisionMeasurement measurement = new VisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                result.getTargets().size(),
                calculateAverageDistance(est, result.getTargets()),
                curStdDevs
            );

            if (consumer != null && measurement.isValid()) {
                consumer.accept(measurement);
            }
        });
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets
    ) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            return;
        }

        var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;

        for (var t : targets) {
            var tagPose = poseEstimator
                .getFieldTags()
                .getTagPose(t.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(
                    estimatedPose
                        .get()
                        .estimatedPose.toPose2d()
                        .getTranslation()
                );
        }

        if (numTags == 0) {
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            return;
        }

        avgDist /= numTags;

        if (numTags > 1) {
            estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
        }

        if (numTags == 1 && avgDist > VisionConstants.MAX_DETECTION_DISTANCE) {
            estStdDevs = VecBuilder.fill(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE
            );
        } else {
            estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));
        }

        curStdDevs = estStdDevs;
    }

    private double calculateAverageDistance(
        EstimatedRobotPose estimate,
        List<PhotonTrackedTarget> targets
    ) {
        double totalDist = 0;
        int validTags = 0;

        for (var t : targets) {
            var tagPose = poseEstimator
                .getFieldTags()
                .getTagPose(t.getFiducialId());
            if (tagPose.isEmpty()) continue;

            validTags++;
            totalDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(
                    estimate.estimatedPose.toPose2d().getTranslation()
                );
        }

        return validTags > 0 ? totalDist / validTags : 0.0;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Transform3d getPosition() {
        return cameraPostion;
    }

    public static class PVCameraConfig {

        private final String name;
        private Transform3d robotToCamera;
        private boolean enabled = true;
        private MeasurementConsumer measurementConsumer;

        public PVCameraConfig(String name) {
            this.name = name;
        }

        public PVCameraConfig withTransform(Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            return this;
        }

        public PVCameraConfig setEnabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        public PVCameraConfig withMeasurementConsumer(
            MeasurementConsumer consumer
        ) {
            this.measurementConsumer = consumer;
            return this;
        }

        public PVCamera build() {
            if (robotToCamera == null) {
                throw new IllegalStateException(
                    "Robot-to-camera transform must be set"
                );
            }
            return new PVCamera(this);
        }
    }
}
