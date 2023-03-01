package frc.robot.vision;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    PhotonCamera camera;
    
public PhotonCamera getCamera() {
      return camera;
    }

    public void setCamera(PhotonCamera camera) {
      this.camera = camera;
    }

}


private void targetDrive(){


    var result = camera.getLatestResult();



    boolean hasTargets = result.hasTargets();
    if(hasTargets){
      
      var target = result.getBestTarget();
      int targetID = target.getFiducialId();
      SmartDashboard.putNumber("targetID", targetID);

      double area = target.getArea();
      SmartDashboard.putNumber("targetArea", area);
     if(area < 5){
        motorFrontLeft.set(ControlMode.PercentOutput, -0.2);
        motorFrontRight.set(ControlMode.PercentOutput, 0.2);

      }
      else{
        motorFrontLeft.set(ControlMode.PercentOutput, 0);
        motorFrontRight.set(ControlMode.PercentOutput, 0);
      }


    }