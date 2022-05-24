package frc.robot.Commands.Auto;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;
public class GenerateLeftTwo extends CommandBase{
    static CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
    static DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts.get(),
                DriveConstants.kvVoltSecondsPerMeter.get(),
                DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
            DriveConstants.kDriveKinematics,
            6);
    static DifferentialDriveKinematicsConstraint differentialDriveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, AutoConstants.kMaxSpeedMetersPerSecond);
    // Create config for trajectory
    
    static TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .addConstraint(differentialDriveKinematicsConstraint)
                .addConstraint(centripetalAccelerationConstraint);
        public static Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                    // Start at p0
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through i0, i1
                    List.of(new Translation2d(0.2, 1), new Translation2d(0.2, 0.2)),
                    // End at p1
                    new Pose2d(.5, 1, new Rotation2d(Math.PI)),
                    // Pass config
                    config);
    public static Command generate(Drivetrain drivetrain){
        // var centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(AutoConstants.maxCentripetalAcceleration);
        // var autoVoltageConstraint =
        //     new DifferentialDriveVoltageConstraint(
        //         new SimpleMotorFeedforward(
        //             DriveConstants.ksVolts.get(),
        //             DriveConstants.kvVoltSecondsPerMeter.get(),
        //             DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
        //         DriveConstants.kDriveKinematics,
        //         6);
        // var differentialDriveKinematicsConstraint = new DifferentialDriveKinematicsConstraint(DriveConstants.kDriveKinematics, AutoConstants.kMaxSpeedMetersPerSecond);
        // // Create config for trajectory
        // TrajectoryConfig config =
        //     new TrajectoryConfig(
        //             AutoConstants.kMaxSpeedMetersPerSecond,
        //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(DriveConstants.kDriveKinematics)
        //         // Apply the voltage constraint
        //         .addConstraint(autoVoltageConstraint)
        //         .addConstraint(differentialDriveKinematicsConstraint)
        //         .addConstraint(centripetalAccelerationConstraint);

        //         config.setReversed(false);

        RamseteCommand ramseteCommand =
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
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}