package frc.robot.Subsystems;

import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class Vision {

    PhotonCamera camera = new PhotonCamera("photonvision");
    
    AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.aprilField,651.25,323.25);
    /* tag id to location
    1,2 = left and right blue source
    3,4 = right and center red speaker
    5 = red amp
    6 = blue amp
    7,8 = center and right blue speaker
    9, 10 = right and left red source
    11, 12, 13 = left right far stage red
    */

}
