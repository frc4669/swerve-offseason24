package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class wraps {@link SwerveDriveOdometry Swerve Drive Odometry} to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is intended to be a
 * drop-in replacement for {@link edu.wpi.first.math.kinematics.SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 */
public class CorrectAxisSwerveOdometry extends SwerveDrivePoseEstimator {
  public static Pose2d CorrectAxis(Pose2d oldPose) {
    return new Pose2d(-oldPose.getX(), oldPose.getY(), oldPose.getRotation());
  }

  /**
   * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision
   * measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
   * meters for x, 0.9 meters for y, and 0.9 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   */
  public CorrectAxisSwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public CorrectAxisSwerveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
    super(kinematics, gyroAngle, modulePositions, CorrectAxis(initialPoseMeters), stateStdDevs, visionMeasurementStdDevs);
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters The position on the field that your robot is at.
   */
  @Override
  public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    super.resetPosition(gyroAngle, modulePositions, CorrectAxis(poseMeters));
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @Override
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return CorrectAxis(super.update(gyroAngle, modulePositions));
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @Override
  public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return CorrectAxis(super.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions));
  }

  @Override
  public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveDriveWheelPositions wheelPositions) {
    return CorrectAxis(super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions));
  }
  
  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  @Override
  public Pose2d getEstimatedPosition() {
    return CorrectAxis(super.getEstimatedPosition());
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link
   *     PoseEstimator#updateWithTime(double,Rotation2d,WheelPositions)} then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.) This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source or
   *     sync the epochs.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(CorrectAxis(visionRobotPoseMeters), timestampSeconds);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link #updateWithTime}, then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *     this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(CorrectAxis(visionRobotPoseMeters), timestampSeconds, visionMeasurementStdDevs);
  }


  
}

