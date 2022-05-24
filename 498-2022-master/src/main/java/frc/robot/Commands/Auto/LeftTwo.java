/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
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


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LeftTwo extends SequentialCommandGroup {
/**
 * @param Limelight enabled
 * @param Score
 * @param Drive through Trench Run,
 * @param Score
 */
  public LeftTwo(Drivetrain drivetrain, Flywheel flywheel, UpperIndex upperIndex, LowerIndex lowerIndex, Wrist wrist, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
     super(
      new ParallelCommandGroup(
           GenerateTrajectory.generate(drivetrain, new Pose2d(0, 0, new Rotation2d(Math.PI)), new Pose2d(.5, 1, new Rotation2d(0)), 
           new Translation2d(0, 1), new Translation2d(0.25, 0.25)),
         new SequentialCommandGroup(
           new ToggleWristOut(wrist),
           new ParallelCommandGroup(
            new TimedIntake(intake, Intake.State.FWD, 3),
            new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 3),
            new TimedUpperIndex(upperIndex, UpperIndex.State.REV, 3)
           ),
          //  new ParallelCommandGroup(
          //   new TimedIntake(intake, Intake.State.FWD, 3),
          //   new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 3),
          //   new TimedUpperIndex(upperIndex, UpperIndex.State.REV, 3)
          //  ),
           new ParallelCommandGroup(
            new ToggleWristIn(wrist),
            new WaitCommand(3)
          ),
          new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
          new ParallelCommandGroup(
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 1),
            new TimedShootCargo(upperIndex, lowerIndex, .3)
          ),
          new ParallelCommandGroup(
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 1),
            new TimedShootCargo(upperIndex, lowerIndex, .3)
          )
         )
      )

   );
      }
}