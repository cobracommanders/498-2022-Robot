/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Auto.Routines;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.Commands.Auto.RamsetA;
import frc.robot.Commands.SloppyAuto.TimedFlywheel;
import frc.robot.Commands.SloppyAuto.TimedIntake;
import frc.robot.Commands.SloppyAuto.TimedLowerIndex;
import frc.robot.Commands.SloppyAuto.TimedShootCargo;
import frc.robot.Commands.SloppyAuto.TimedUpperIndex;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;
import frc.robot.util.FieldConstants;
import frc.robot.util.Util;

public class RightThree extends SequentialCommandGroup {
  private static Rotation2d cargoDangleOfApproach = Rotation2d.fromDegrees(180);
  private static Pose2d cargoDPose =
      new Pose2d(FieldConstants.cargoD.getTranslation(), cargoDangleOfApproach)
          .transformBy(Util.Geometry.transformFromTranslation(0, -Units.inchesToMeters(13)));

  private static Trajectory leg1 =
      RamsetA.makeTrajectory(
          0.0,
          List.of(
            FieldConstants.StartingPoints.fenderB, 
            cargoDPose),
          0.0,
          Units.feetToMeters(8),
          false);


  public RightThree(Drivetrain drivetrain, Flywheel flywheel, UpperIndex upperIndex, LowerIndex lowerIndex, Wrist wrist, Intake intake) {
    Command collectCargo =
        new ParallelDeadlineGroup(
          RamsetA.RamseteSchmoove(leg1, drivetrain),
          new SequentialCommandGroup(
           new WaitCommand(1),
           new ToggleWristOut(wrist),
           new ParallelCommandGroup(
            new TimedIntake(intake, Intake.State.FWD, 5),
            new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 5),
            new TimedUpperIndex(upperIndex, UpperIndex.State.REV, 5)
           ),
           new ParallelCommandGroup(
           new ToggleWristIn(wrist),
           new TimedFlywheel(flywheel, Flywheel.State.FULL, 2.5)
          )
         )
        );
    Command scoreAllBalls =
          new SequentialCommandGroup(
            new ParallelCommandGroup(
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 1),
                new TimedShootCargo(upperIndex, lowerIndex, .3)
            ),
            new ParallelCommandGroup(
              new TimedFlywheel(flywheel, Flywheel.State.FULL, 1),
              new TimedShootCargo(upperIndex, lowerIndex, .3)
            )
          );
    addCommands(
        collectCargo,
        scoreAllBalls
        );
  }
}
