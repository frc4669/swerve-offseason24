package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.frc4669;
import frc.robot.frc4669.Logging;
import frc.robot.subsystems.SwerveDrivetrain;

public class PathFllowing {
    // go straigh to a cordinate specified on NetworkTables (Note: No obstacle advoidsion)
    public static Command GoToLocation(Vision vision, SwerveDrivetrain swerveDrivetrain) {
        var xOpt = vision.GetCommandParam("GoToLocation", "x");
        var yOpt = vision.GetCommandParam("GoToLocation", "y"); 
        if (xOpt.isEmpty() || yOpt.isEmpty()) {
            Logging.error("GoToLocation called without valid x and y cordinates."); 
            return Commands.none();
        } 
        double tarX = xOpt.get().getDouble();
        double tarY = yOpt.get().getDouble();
        
        Pose2d currentPose = swerveDrivetrain.getRobotPose(); 
        Pose2d targetPose = new Pose2d(tarX, tarY, Rotation2d.fromDegrees(0)); 
        
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(targetPose); 
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(Constants.Auto.kAutoMaxVelocity, Constants.Auto.kAutoMaxAccel, Constants.Auto.kAutoMaxAngularVelocity, Constants.Auto.kAutoMaxAngularAccel), 
            new GoalEndState(0, currentPose.getRotation()),
            false
        );

        Logging.info("GoToLocation " + tarX + " " + tarY);
        return AutoBuilder.followPath(path); 
    }
}
