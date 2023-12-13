//FIX ME --- MODULE POSITION!!!!!!!!!

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();
    
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions);


    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  /**
   * Updates the swerve drivetrain with the specified values.
   * @param velocity
   * @param rotation The angular velocity, in radians per second.
   * @param fieldRelative 
   * @param isOpenLoop 
   */
  public void drive(Translation2d velocity, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), rotation, getYaw()
            )
          : new ChassisSpeeds(velocity.getX(), velocity.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Resets the robot's position on the field.
   * @param pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    }, pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * Resets the yaw value of the gyroscope to zero. 
   */
  public void zeroGyro() {
    gyro.setYaw(0);
  }

  /**
   * Gets the current yaw of the gyro, relative to when we last zeroed it.  
   * Positive Z/yaw is left.
   * @return
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
      swerveModules[0].getPosition(), 
      swerveModules[1].getPosition(), 
      swerveModules[2].getPosition(), 
      swerveModules[3].getPosition()
    });
    field.setRobotPose(getPose());

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
