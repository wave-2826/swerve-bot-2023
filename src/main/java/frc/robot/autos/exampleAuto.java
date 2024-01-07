package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class exampleAuto extends SequentialCommandGroup {
    /**
   * A supplier for X velocity values.
   * When constructing this command, we pass a lambda expression to return the current X velocity. This supplier provides that.
   */
  private DoubleSupplier xVelocitySupplier;
  /**
   * A supplier for Y velocity values.
   * When constructing this command, we pass a lambda expression to return the current Y velocity. This supplier provides that.
   */
  private DoubleSupplier yVelocitySupplier;
  /**
   * A supplier for rotation values.
   * When constructing this command, we pass a lambda expression to return the current rotation. This supplier provides that.
   */
  private DoubleSupplier rotationSupplier;
  /**
   * A supplier for if we're currently field relative.
   * When constructing this command, we pass a lambda expression to return the current field relative mode. This supplier provides that.
   */
  private BooleanSupplier fieldRelativeSupplier;

  /**
   * Limits the rate of change of the swerve drive's X velocity. Velocity is in meters per second.
   */
  private SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(3.0);
  /**
   * Limits the rate of change of the swerve drive's Y velocity. Velocity is in meters per second.
   */
  private SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(3.0);
  /**
   * Limits the rate of change of the swerve drive's rotation velocity. Rotation is in radians per second.
   */
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  
  PIDController anglePID = new PIDController(1.0 / 10.0, 0, 0.005);

  public exampleAuto(Swerve swerveSubsystem) {
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         .setKinematics(Constants.Swerve.swerveKinematics);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // // Pass through these two interior waypoints, making an 's' curve path
    //         // List.of(new Translation2d(1/2, 1/2), new Translation2d(2/2, -1/2)),
    //         List.of(),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         // new Pose2d(3/2, 0, new Rotation2d(0)),
    //         new Pose2d(1, 0, new Rotation2d(0)),
    //         config);

    // var thetaController =
    //     new ProfiledPIDController(
    //         Constants.AutoConstants.kPThetaController,
    //         0,
    //         0,
    //         Constants.AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    //     new SwerveControllerCommand(
    //         exampleTrajectory,
    //         s_Swerve::getPose,
    //         Constants.Swerve.swerveKinematics,
    //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         s_Swerve::setModuleStates,
    //         s_Swerve);

    // addCommands(
    //     new WaitCommand(2),
    //     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
    //     swerveControllerCommand,
    //     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose()))
    // );

    // This is a temporary implementation of a simple AprilTag following procedure.
    // Using the LimelightHelpers library provided by Limelight, we get the target X,
    // Y and area (only X is used) and the camera space (meaning relative to the limelight on the robot)
    // 3D position of the tracked tag.
    // This code uses the result of pipeline 0 because we don't explicitly select a pipeline
    // and that is the default. Pipeline 0 is configured to track the FRC AprilTags.
    // To follow the AprilTag, we do 2 separate things:
    // - Rotate the robot: based on the X position (which is relative to the center of the camera),
    //   change the angular velocity of the drivebase to move in the direction of the target.
    // - Move the robot forward and backward: based on the distane from the camera plane, which is
    //   the Z position of the target, find a difference between that and a target distance (1 meter)
    //   and move the robot forward and backward depening on if that target distance is positive
    //   or negative.
    // Under the hood, the LimelightHelpers library uses the FRC NetworkTables protocol; Limelight supports
    // many different protocols but NetworkTables is what they recommend and use. The Limelight broadcasts
    // these relavant values onto the table with the hostname of the limelight ("limelight" for ours).
    //
    // See the documentation at https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    // for other networktables values.
    
    //read values periodically
    double x = LimelightHelpers.getTX("limelight"); // degrees
    double y = LimelightHelpers.getTY("limelight"); // degrees
    double area = LimelightHelpers.getTA("limelight");
    double[] targetposeCameraspace = LimelightHelpers.getTargetPose_CameraSpace("limelight");
    
    //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);


    boolean seeingTarget = LimelightHelpers.getTV("limelight");
    double targetX = targetposeCameraspace[0];
    double targetY = targetposeCameraspace[1];
    double targetZ = targetposeCameraspace[2];

    double targetDistance = 1.0; // meters
    double distanceDifference = targetZ - targetDistance; // How far we need to move
    if(Math.abs(distanceDifference) < 0.05) distanceDifference = 0; // To avoid jittering


    /* Get Values, Deadband */
    double xVelocity = xVelocityLimiter.calculate(MathUtil.applyDeadband(xVelocitySupplier.getAsDouble(), Constants.Swerve.stickDeadband));
    double yVelocity = yVelocityLimiter.calculate(MathUtil.applyDeadband(yVelocitySupplier.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = rotationLimiter.calculate(MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.Swerve.stickDeadband));

    if(seeingTarget) xVelocity += 0.08 * Math.signum(distanceDifference);

    // if(x == 0.0) System.out.println("X is 0!");

    double trackingAngularVelocity = Math.min(anglePID.calculate(x, 0), 4);
    
    // SmartDashboard.putNumber("PID Value", currentAngle);

    /* Drive */
    swerveSubsystem.drive(
        new Translation2d(xVelocity, yVelocity).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity + trackingAngularVelocity,
        // fieldRelativeSupplier.getAsBoolean(),
        false,
        false);
  }
}
