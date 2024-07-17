

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

  private NetworkTableInstance nt;
  private NetworkTable visionTable;  
  private NetworkTable cncTable;
  
  private NetworkTable robotPosTable; 

  private Map<String, Command> cmdMap; 

  public Vision() {
    this.nt = NetworkTableInstance.getDefault(); 
    var smartDashboardTable = nt.getTable("SmartDashboard"); 
    this.visionTable = smartDashboardTable.getSubTable("Vision"); 
    this.cncTable = smartDashboardTable.getSubTable("C&C"); 
    this.robotPosTable = visionTable.getSubTable("RobotPos"); 

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
    var x = this.robotPosTable.getValue("x");
    var y = this.robotPosTable.getValue("y");
    var r = this.robotPosTable.getValue("r");
    var ts = this.robotPosTable.getValue("ts"); 
     
    // make sure the values actually exist
    if (!x.isDouble() || !y.isDouble() || !r.isDouble() || !ts.isDouble()) return Optional.empty(); 
    return Optional.of(new VisionRobotPos(x.getDouble(), y.getDouble(), r.getDouble(), ts.getDouble())); 
  }

  // get parameters of a command
  public NetworkTableValue GetCommandParam(String cmdName, String paramName) {
    return this.cncTable.getSubTable(cmdName).getValue(paramName); 
  }
}
