// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import org.photonvision.PhotonCamera;
//import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSub extends SubsystemBase {
  PhotonCamera limelight;
  /** Creates a new ExampleSubsystem. 
   * @cameraName the name from the photonvision gui
  */
  public PhotonSub(String cameraName) {
    limelight = new PhotonCamera(cameraName);
    PortForwarder.add(5800, "photonvision.local", 5800);
  } 

  public PhotonPipelineResult getTargets() {
    return limelight.getLatestResult();
  }

  public void getInfo() {
    PhotonPipelineResult pipelineResult = getTargets();
    if ( pipelineResult.hasTargets() ) {
      // yaw is left and right
      // skew is roll side to side
      // pitch is up and down
      PhotonTrackedTarget trackedTarget = pipelineResult.getBestTarget();
      System.out.println("Fiducial ID: " + trackedTarget.getFiducialId());
      System.out.println("Yaw: " + trackedTarget.getYaw());
      System.out.println("Pitch: " + trackedTarget.getPitch());
      System.out.println("Skew: " + trackedTarget.getSkew());

    } else {
      System.out.println("camera not see anythig\n");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
