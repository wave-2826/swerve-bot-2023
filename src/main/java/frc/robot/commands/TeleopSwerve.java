package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve swerveSubsystem;
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

  public TeleopSwerve(
      Swerve swerveSubsystem,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double xVelocity = xVelocityLimiter.calculate(MathUtil.applyDeadband(xVelocitySupplier.getAsDouble(), Constants.Swerve.stickDeadband));
    double yVelocity = yVelocityLimiter.calculate(MathUtil.applyDeadband(yVelocitySupplier.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = rotationLimiter.calculate(MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    swerveSubsystem.drive(
        new Translation2d(xVelocity, yVelocity).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        fieldRelativeSupplier.getAsBoolean(),
        false);
  }
}
