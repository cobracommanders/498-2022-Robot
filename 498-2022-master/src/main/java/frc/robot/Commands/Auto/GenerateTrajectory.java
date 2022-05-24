package frc.robot.Commands.Auto;

import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;
public class GenerateTrajectory extends CommandBase{
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

    public static Command generate(Drivetrain drivetrain, Pose2d p0, Pose2d p1, Translation2d i0, Translation2d i1){
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
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at p0
                p0,
                // Pass through i0, i1
                List.of(i0, i1),
                // End at p1
                p1,
                // Pass config
                config);
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
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
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}