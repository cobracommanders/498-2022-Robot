package frc.robot.Commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class RamsetA extends SequentialCommandGroup {

  protected Trajectory autoTrajectory;

  public static Trajectory makeTrajectory(
      double startVelocity, List<Pose2d> waypoints, double endVelocity, boolean reversed) {
    return makeTrajectory(
        startVelocity, waypoints, endVelocity, Constants.AutoConstants.kMaxSpeedMetersPerSecond, reversed);
  }

  public static Trajectory makeTrajectory(
      double startVelocity,
      List<Pose2d> waypoints,
      double endVelocity,
      double maxVelocity,
      boolean reversed) {
    CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts.get(),
                    DriveConstants.kvVoltSecondsPerMeter.get(),
                    DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
                DriveConstants.kDriveKinematics,
                10);
    DifferentialDriveKinematicsConstraint differentialDriveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, AutoConstants.kMaxSpeedMetersPerSecond);
        
  return TrajectoryGenerator.generateTrajectory(
        waypoints,
        new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .setStartVelocity(startVelocity)
                .setEndVelocity(endVelocity)
                .addConstraint(autoVoltageConstraint)
                .addConstraint(differentialDriveKinematicsConstraint)
                .addConstraint(centripetalAccelerationConstraint));
    }
  public static Command RamseteSchmoove(Trajectory autoTrajectory, Drivetrain driveSubsystem) {
    RamseteCommand ramsete =
        new RamseteCommand(
            autoTrajectory,
            driveSubsystem::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts.get(),
                Constants.DriveConstants.kvVoltSecondsPerMeter.get(),
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
            Constants.DriveConstants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel.get(), 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel.get(), 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              driveSubsystem.resetOdometry(autoTrajectory.getInitialPose());
            }),
        ramsete,
        new InstantCommand(
            () -> {
              driveSubsystem.tankDriveVolts(0, 0);
            }));
        }
    }
