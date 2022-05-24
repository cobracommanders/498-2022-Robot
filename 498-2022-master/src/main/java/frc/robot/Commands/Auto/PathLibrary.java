package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.io.IOException;
import java.nio.file.Path;
public class PathLibrary extends CommandBase{
    public static Command findPath(Drivetrain drivetrain, String pathDirectory){
        Trajectory m_trajectory = trajectory(pathDirectory);//"output/test.wpilib.json"
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                m_trajectory,
                drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts.get(),
                    DriveConstants.kvVoltSecondsPerMeter.get(),
                    DriveConstants.kaVoltSecondsSquaredPerMeter.get()),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                drivetrain.leftController,//new PIDController(DriveConstants.kPDriveVel.get(), DriveConstants.kI, DriveConstants.kD),
                drivetrain.rightController,//new PIDController(DriveConstants.kPDriveVel.get(), DriveConstants.kI, DriveConstants.kD),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(m_trajectory.getInitialPose());
        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
    static Trajectory trajectory(String trajectoryJSON) {
        //String trajectoryJSON = "paths/circle.wpilib.json";
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, e.getStackTrace());
        }
        return trajectory;
    }
}