package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class RamseteCommandGenerate extends CommandBase{
  private final Drivetrain drivetrain;
  double s, r;
  private final int path;
  RamseteCommand ramseteCommand;
  public RamseteCommandGenerate(Drivetrain drivetrain, int path) {
    this.drivetrain = drivetrain;
    this.path = path;
    addRequirements(this.drivetrain);
  }
  CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              DriveConstants.ksVolts.get(),
              DriveConstants.kvVoltSecondsPerMeter.get(),
              DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
          DriveConstants.kDriveKinematics,
          6);
  DifferentialDriveKinematicsConstraint differentialDriveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, AutoConstants.kMaxSpeedMetersPerSecond);
  // Create config for trajectory
  
  TrajectoryConfig config =
          new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(DriveConstants.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint)
              .addConstraint(differentialDriveKinematicsConstraint)
              .addConstraint(centripetalAccelerationConstraint);
  Trajectory trajectory;
  Trajectory rightThreeTrajectory =
    TrajectoryGenerator.generateTrajectory(
                  // Start at p0
                  new Pose2d(0, 0, new Rotation2d(Math.PI)),
                  // Pass through i0, i1
                  List.of(new Translation2d(-2, 2.2), new Translation2d(-2.7, -1.8)),
                  // End at p1
                  new Pose2d(.5, -.8, new Rotation2d(0)),
                  // Pass config
                  config);
  Trajectory leftTwoTrajectory =
  TrajectoryGenerator.generateTrajectory(
                // Start at p0
                new Pose2d(1, 1, new Rotation2d(1)),
                // Pass through i0, i1
                List.of(new Translation2d(-1, 1)),
                // End at p1
                new Pose2d(.5, 1, new Rotation2d(Math.PI)),
                // Pass config
                config);
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    switch (path) {
      case 1:
        trajectory = rightThreeTrajectory;
      case 2:
        trajectory = leftTwoTrajectory;
    }
    ramseteCommand =
    new RamseteCommand(
        trajectory,
        drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        drivetrain.feedForward,
        DriveConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        drivetrain.leftController,
        drivetrain.rightController,
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain);
        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(trajectory.getInitialPose());
        // Run path following command, then stop at the end.
        ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    SmartDashboard.putNumber("Left Drive Velocity", drivetrain.leftWheelSpeedMetersPerSecond());
    SmartDashboard.putNumber("Right Drive Velocity", drivetrain.rightWheelSpeedMetersPerSecond());
    SmartDashboard.putNumber("Drive Heading", drivetrain.getHeading().getDegrees());
    SmartDashboard.putNumber("Drive Total Velocity", drivetrain.getAbsoluteVelocity());
  }
  @Override 
  public boolean isFinished(){
    return ramseteCommand.isFinished();
  }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
