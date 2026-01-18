// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final String cameraName;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.cameraName = name;
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());

    super.updateInputs(inputs);

    logRays(poseSupplier.get(), inputs);
  }

  // method for logging the rays from camera to april tag
  private void logRays(Pose2d robotPose2d, VisionIOInputs inputs) {
    Pose3d fieldToCamera = new Pose3d(robotPose2d).transformBy(robotToCamera);

    List<Pose3d> rayPoints = new ArrayList<>();

    for (int tagId : inputs.tagIds) {
      var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
      if (tagPoseOpt.isEmpty()) continue;

      Pose3d fieldToTag = tagPoseOpt.get();

      // each pair = one ray
      rayPoints.add(fieldToCamera);
      rayPoints.add(fieldToTag);
    }

    Logger.recordOutput("Vision/SimRays/" + cameraName, rayPoints.toArray(new Pose3d[0]));
  }
}
