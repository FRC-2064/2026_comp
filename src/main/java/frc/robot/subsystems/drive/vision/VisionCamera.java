package frc.robot.subsystems.drive.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionCamera {
    void periodic();

    String getName();

    boolean isEnabled();

    void setEnabled(boolean enabled);

    Matrix<N3, N1> getEstimationStdDevs();

    class VisionMeasurement {

        public final Pose2d pose;
        public final double timestampSeconds;
        public final int tagCount;
        public final double averageDistance;
        public final Matrix<N3, N1> stdDevs;

        public VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            int tagCount,
            double averageDistance,
            Matrix<N3, N1> stdDevs
        ) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.tagCount = tagCount;
            this.averageDistance = averageDistance;
            this.stdDevs = stdDevs;
        }

        public boolean isValid() {
            return stdDevs != null && stdDevs.get(0, 0) != Double.MAX_VALUE;
        }
    }

    @FunctionalInterface
    interface MeasurementConsumer {
        void accept(VisionMeasurement measurement);
    }
}
