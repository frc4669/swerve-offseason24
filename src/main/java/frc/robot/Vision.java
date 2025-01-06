

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.frc4669;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;

public class Vision {

  private NetworkTableInstance m_nt;
  private NetworkTable m_visionTable;  
  private NetworkTable m_cncTable;
  
  private NetworkTable m_robotPosTable; 

  public Vision() {
    this.m_nt = NetworkTableInstance.getDefault(); 
    var smartDashboardTable = m_nt.getTable("SmartDashboard"); 
    this.m_visionTable = m_nt.getTable("Vision"); 
    this.m_cncTable = smartDashboardTable.getSubTable("C&C"); 
    this.m_robotPosTable = m_visionTable.getSubTable("RobotPos"); 
  }

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

  // get robot absoulote position from NetworkTables
  public Optional<VisionRobotPos> GetVisionRobotPos() {
    var x = this.m_robotPosTable.getValue("x");
    var y = this.m_robotPosTable.getValue("y");
    var r = this.m_robotPosTable.getValue("r");
    var ts = this.m_robotPosTable.getValue("ts"); 
     
    // make sure the values actually exist
    if (!x.isDouble() || !y.isDouble() || !r.isDouble() || !ts.isDouble()) return Optional.empty();
    
    // make sure a valid timestamp that's not negative or zero 
    if (ts.getDouble() < 1) return Optional.empty();
    return Optional.of(new VisionRobotPos(x.getDouble(), y.getDouble(), r.getDouble(), ts.getDouble())); 
  }

  // get parameters of a command
  public Optional<NetworkTableValue> GetCommandParam(String cmdName, String paramName) {
    var value = this.m_cncTable.getSubTable(cmdName).getValue(paramName); 
    if (!value.isValid()) return Optional.empty(); 
    return Optional.of(value);
  }
}
