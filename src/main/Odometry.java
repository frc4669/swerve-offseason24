
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Odometry extends SwerveDrivePoseEstimator {
    public Odometry(SwerveDrivePoseEstimatorSwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters); 
    }

    
}
