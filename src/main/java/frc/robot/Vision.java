

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;

import java.security.Key;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.frc4669;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;

public class Vision {

  private NetworkTableInstance m_nt;
  private NetworkTable m_visionTable;  
  private NetworkTable m_cncTable;
  
  private NetworkTable m_robotPosTable; 

  private Map<String, Command> cmdMap; 

  public Vision() {
    this.m_nt = NetworkTableInstance.getDefault(); 
    var smartDashboardTable = m_nt.getTable("SmartDashboard"); 
    this.m_visionTable = smartDashboardTable.getSubTable("Vision"); 
    this.m_cncTable = smartDashboardTable.getSubTable("C&C"); 
    this.m_robotPosTable = m_visionTable.getSubTable("RobotPos"); 

    cmdMap = new HashMap<String, Command>(); 
  }

  public void register(String name, Command robotCmd) {
    this.cmdMap.put(name, robotCmd); 
  }

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
    return Optional.of(new VisionRobotPos(x.getDouble(), y.getDouble(), r.getDouble(), ts.getDouble())); 
  }

  // get parameters of a command
  public Optional<NetworkTableValue> GetCommandParam(String cmdName, String paramName) {
    var value = this.m_cncTable.getSubTable(cmdName).getValue(paramName); 
    if (!value.isValid()) return Optional.empty(); 
    return Optional.of(value);
  }
}