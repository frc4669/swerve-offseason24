

package frc.robot;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.frc4669;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.wpilibj.Filesystem;

public class Vision {
  private NetworkTableInstance m_nt;
  private NetworkTable m_visionTable;  
  private NetworkTable m_camerasTable; 
  private NetworkTable m_cncTable;
  
  private NetworkTable m_robotPosTable; 

  // Pose2d extended with Vision specific parameters
  public class VisionRobotPos extends Pose2d { 
    public double ts; 
    public VisionRobotPos(double x, double y, double r_deg, double ts) {
      super(x, y, Rotation2d.fromDegrees(r_deg));
      this.ts = ts;  
    }
    public VisionRobotPos() {
      super(); 
    }
  }

  public Vision() {
    this.m_nt = NetworkTableInstance.getDefault(); 
    var smartDashboardTable = m_nt.getTable("SmartDashboard"); 
    this.m_visionTable = m_nt.getTable("Vision"); 
    m_camerasTable = m_visionTable.getSubTable("Cameras"); 

    this.m_cncTable = smartDashboardTable.getSubTable("C&C"); 
    this.m_robotPosTable = m_visionTable.getSubTable("RobotPos"); 
  }

  // Get Tag pose relative to a camera (NOTE: This does not correct for camera offset to robot center)
  // tag detected more than maxAgeMS millseconds before will not be returned as a result 
  public Optional<Pose2d> GetTagPoseRelativeToCamera(String cameraName, Integer tagID, int maxAgeMS) {
    if (! m_camerasTable.containsSubTable(cameraName)) {
      System.err.println("VISION: Camera " + cameraName + " not found!");
      return Optional.empty();
    }

    var cameraTable = m_camerasTable.getSubTable(cameraName); 
    if (! cameraTable.containsSubTable(tagID.toString())) Optional.empty();

    var tagTable = cameraTable.getSubTable(tagID.toString()); 

    try {
      var ts = tagTable.getValue("ts").getDouble();
      if ((NetworkTablesJNI.now() - ts) > maxAgeMS * 1000) {
        return Optional.empty();
      }

      var tvec = tagTable.getValue("tvec").getDoubleArray(); 
      var rvec = tagTable.getValue("rvec").getDoubleArray();

      Pose2d retPose = new Pose2d(
        tvec[2], 
        tvec[0],
        Rotation2d.fromDegrees(rvec[1])
      );

      return Optional.of(retPose);
    } catch (Exception e) {
      System.err.println("VISION: Failed to read");
      return Optional.empty();
    }
  }
  // version with a default 500 ms maxAge
  public Optional<Pose2d> GetTagPoseRelativeToCamera(String cameraName, Integer tagID) { return GetTagPoseRelativeToCamera(cameraName, tagID, 500); }


  // get robot absoulote position from NetworkTables
  public Optional<VisionRobotPos> GetVisionRobotPos() {
    var x = this.m_robotPosTable.getValue("x");
    var y = this.m_robotPosTable.getValue("y");
    var r = this.m_robotPosTable.getValue("r");
    var ts = this.m_robotPosTable.getValue("ts"); 
    
    // make sure the values actually exist
    if (!x.isDouble() || !y.isDouble() || !r.isDouble() || !ts.isInteger()) return Optional.empty();
    
    // make sure a valid timestamp that's not negative or zero 
    if (ts.getInteger() < 1) return Optional.empty();
    return Optional.of(new VisionRobotPos(x.getDouble(), y.getDouble(), r.getDouble(), ts.getInteger())); 
  }

  // get parameters of a command
  public Optional<NetworkTableValue> GetCommandParam(String cmdName, String paramName) {
    var value = this.m_cncTable.getSubTable(cmdName).getValue(paramName); 
    if (!value.isValid()) return Optional.empty(); 
    return Optional.of(value);
  }
}
