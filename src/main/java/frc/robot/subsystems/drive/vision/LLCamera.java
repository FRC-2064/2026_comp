package frc.robot.subsystems.drive.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class LLCamera implements VisionCamera {

    private final String name;
    private final Transform3d robotToCamera;
    private final MeasurementConsumer consumer;
    private final boolean useMegaTag2;
    private boolean enabled;
    private Matrix<N3, N1> curStdDevs;

    private LLCamera(LLCameraConfig config) {
        this.name = config.name;
        this.robotToCamera = config.robotToCamera;
        this.enabled = config.enabled;
        this.useMegaTag2 = config.useMT2;
        this.consumer = config.consumer;
        this.curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (!LimelightHelpers.validPoseEstimate(estimate)) {
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            return;
        }

        if (estimate.tagCount < VisionConstants.MIN_TAG_COUNT) {
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            return;
        }

        curStdDevs = calculateStdDevs(estimate);

        VisionMeasurement measurement = new VisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            estimate.tagCount,
            estimate.avgTagDist,
            curStdDevs
        );
        if (consumer != null && measurement.isValid()) {
            consumer.accept(measurement);
        }
    }

    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        int numTags = estimate.tagCount;
        double avgDist = estimate.avgTagDist;

        Matrix<N3, N1> estStdDevs =
            numTags > 1
                ? VisionConstants.MULTI_TAG_STD_DEVS
                : VisionConstants.SINGLE_TAG_STD_DEVS;

        if (numTags == 1 && avgDist > VisionConstants.MAX_DETECTION_DISTANCE) {
            return VecBuilder.fill(
                Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE
            );
        }

        estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));

        return estStdDevs;
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

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    public boolean isUsingMT2() {
        return useMegaTag2;
    }

    public static class LLCameraConfig {

        private final String name;
        private Transform3d robotToCamera;
        private boolean enabled = true;
        private boolean useMT2 = true;
        private MeasurementConsumer consumer;

        public LLCameraConfig(String name) {
            this.name = name;
        }

        public LLCameraConfig withTransform(Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            return this;
        }

        public LLCameraConfig withMegaTag2(boolean useMT2) {
            this.useMT2 = useMT2;
            return this;
        }

        public LLCameraConfig setEnabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        public LLCameraConfig withMeasurementConsumer(
            MeasurementConsumer consumer
        ) {
            this.consumer = consumer;
            return this;
        }

        public LLCamera build() {
            if (robotToCamera == null) {
                throw new IllegalStateException("Camera pose required");
            }
            return new LLCamera(this);
        }
    }
}
