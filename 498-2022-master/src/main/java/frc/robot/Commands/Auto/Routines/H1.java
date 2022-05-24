package frc.robot.Commands.Auto.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.StoreCargo;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.Commands.Auto.PathLibrary;
import frc.robot.Commands.SloppyAuto.TimedFlywheel;
import frc.robot.Commands.SloppyAuto.TimedIntake;
import frc.robot.Commands.SloppyAuto.TimedLowerIndex;
import frc.robot.Commands.SloppyAuto.TimedShootCargo;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class H1 extends SequentialCommandGroup {
  public H1(Drivetrain drivetrain, Wrist wrist, Flywheel flywheel, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex) {
     super(
        new ParallelCommandGroup(
          //GenerateTrajectory.generate(drivetrain, new Pose2d(0, 0, new Rotation2d(0)), p1, i0, i1)
          new SequentialCommandGroup(
          //PathLibrary.findPath(drivetrain, "output/H.1.1.wpilib.json"),
          PathLibrary.findPath(drivetrain, "output/H.1.3.wpilib.json")
          ),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new ParallelCommandGroup(
              new ToggleWristOut(wrist),
              new TimedIntake(intake, Intake.State.FWD, 3),
              new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 3),
              new TimedFlywheel(flywheel, Flywheel.State.REV, 3)
            ),
              new ToggleWristIn(wrist),
              new StoreCargo(upperIndex, lowerIndex),
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 3),
            new ParallelCommandGroup(
              new TimedFlywheel(flywheel, Flywheel.State.FULL, 3),
              new TimedShootCargo(upperIndex, lowerIndex, .5)
            ),
            new ParallelCommandGroup(
              new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
              new TimedShootCargo(upperIndex, lowerIndex, .5)
            )
          )
        )
     );
  }
}
