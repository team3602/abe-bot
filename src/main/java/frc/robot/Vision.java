package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import static frc.robot.Constants.VisionConstants.*;





public class Vision {

    private static final String visionTag = null;

    public final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final static PhotonCamera photonCamera = new PhotonCamera(visionTag);
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, kRobotToCamera);
  
    private double lastEstimateTimestamp = 0.0;
  
    public Vision() {
      configVision();
    }
  
    public static PhotonPipelineResult getLatestResult() {
      return photonCamera.getLatestResult();
    }
  
  
    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
      var visionEstimate = photonPoseEstimator.update();
      double latestTimestamp = getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;
  
      if (newResult) {
        lastEstimateTimestamp = latestTimestamp;
      }
  
      return visionEstimate;
    }
  
    public double getTargetHeight() {
      double targetHeight;
      
      var result = getLatestResult();
  
      if (result.hasTargets()) {
        targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
      } else {
        targetHeight = 0.0;
      }
  
      return targetHeight;
    }
  
    public double getTargetDistance() {
      double targetDistance;
  
      var result = getLatestResult();
  
      if (result.hasTargets()) {
        targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
            kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
      } else {
        targetDistance = 0.0;
      }
  
      return targetDistance;
    }
  
    private void configVision() {
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    
}
