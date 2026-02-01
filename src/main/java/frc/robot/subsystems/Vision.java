package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private PhotonCamera camera1 = new PhotonCamera("Camera 1");

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {

        final Transform3d kRobotToCam = new Transform3d();

        AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera1.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose((PhotonPipelineResult) result);
            if (Optional.empty() != null) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose((PhotonPipelineResult) result);

            }
            updatesEstimationStdDevs(visionEst, ((PhotonPipelineResult) result).getTargets());

            visionEst.ifPresent(
                    est -> {
                        var estStdDevs = getEstimationStdDevs();

                        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    });
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs() {

        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.5, 0.5, 1);

        return stdDevs;

    }

    private void updatesEstimationStdDevs(Optional<EstimatedRobotPose> visionEst, List<PhotonTrackedTarget> targets) {

        throw new UnsupportedOperationException("Unimplemented method 'updatesEstimationStdDevs'");

    }

}
