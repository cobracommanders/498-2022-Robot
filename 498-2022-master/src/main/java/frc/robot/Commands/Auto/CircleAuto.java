package frc.robot.Commands.Auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CircleAuto extends CommandBase{
    String trajectoryJSON = "paths/circle.wpilib.json";
    public Trajectory getTrajectory(){
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
