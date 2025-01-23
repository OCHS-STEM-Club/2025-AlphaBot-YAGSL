/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.Subsystems;

 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
 import edu.wpi.first.wpilibj2.command.Command;
 
 import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonPipelineResult;
 import swervelib.telemetry.SwerveDriveTelemetry;
 
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 
 import frc.robot.Constants;
 import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
 
 public class Vision {
    // Back Left Camera
     private final PhotonCamera kBackLeftCamera;
     private final PhotonPoseEstimator backLeftCameraPoseEstimator;

     private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
     private double lastEstTimestamp = 0;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
 
     public Vision() {

        // Back left Camera
        kBackLeftCamera = new PhotonCamera("Center");

        backLeftCameraPoseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                 VisionConstants.BACK_LEFT_CAM_TO_CENTER);

        backLeftCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

     }

     public PhotonPipelineResult getBackLeftLatestResults() {
         return kBackLeftCamera.getLatestResult();
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getBackLeftGlobalEstimatedPose() {
         var visionEst = backLeftCameraPoseEstimator.update(getBackLeftLatestResults());
         double latestTimestamp = kBackLeftCamera.getLatestResult().getTimestampSeconds();
         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
         if (SwerveDriveTelemetry.isSimulation) {
             visionEst.ifPresentOrElse(
                     est ->
                             getSimDebugField()
                                     .getObject("VisionEstimation")
                                     .setPose(est.estimatedPose.toPose2d()),
                     () -> {
                         if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                     });
         }
         if (newResult) lastEstTimestamp = latestTimestamp;
         return visionEst;
     }
 
     /**
      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.

      getEstimationStdDevs
      */
     public Matrix<N3, N1> getBackleftEstimationStdDevs(Pose2d estimatedPose) {
         var estStdDevs = VisionConstants.kSingleTagStdDevs;
         var targets = getBackLeftLatestResults().getTargets();
         int numTags = 0;
         double avgDist = 0;
         for (var tgt : targets) {
             var tagPose = backLeftCameraPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty()) continue;
             numTags++;
             avgDist +=
                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
         }
         if (numTags == 0) return estStdDevs;
         avgDist /= numTags;
         // Decrease std devs (standard deviation) if multiple targets are visible
         if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
         // Increase std devs based on (average) distance
         if (numTags == 1 && avgDist > 4)
             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
 
         return estStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (SwerveDriveTelemetry.isSimulation) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         return visionSim.getDebugField();
     }
 
     public void gettagdistance(){
         //
     }
 
     public Command score(){
         return null;
     }
 }