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

public class T1 extends SequentialCommandGroup {
  public T1(Drivetrain drivetrain, Wrist wrist, Flywheel flywheel, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex) {
     super(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            PathLibrary.findPath(drivetrain, "output/T.1.1.wpilib.json"),
            PathLibrary.findPath(drivetrain, "output/T.1.2.wpilib.json"),
            PathLibrary.findPath(drivetrain, "output/T.1.3.wpilib.json")
          ),
          new SequentialCommandGroup(
            new ParallelCommandGroup(
              new ToggleWristOut(wrist),
              new TimedIntake(intake, Intake.State.FWD, 2),
              new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 2)
            ),//2
            new ParallelCommandGroup(
              new ToggleWristIn(wrist),
              new StoreCargo(upperIndex, lowerIndex)
            ),//3.5
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
            new ParallelCommandGroup(
              new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
              new TimedShootCargo(upperIndex, lowerIndex, 1)
            ),//7.5
            new ParallelCommandGroup(
              new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
              new TimedShootCargo(upperIndex, lowerIndex, 1)
            ),//10.5
            new WaitCommand(1),
            new ParallelCommandGroup(
              new ToggleWristOut(wrist),
              new TimedIntake(intake, Intake.State.FWD, 2),
              new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 2)
            ),//14
            new ParallelCommandGroup(
              new ToggleWristIn(wrist),
              new StoreCargo(upperIndex, lowerIndex)
            ),
            new ParallelCommandGroup(
              new ToggleWristOut(wrist),
              new TimedIntake(intake, Intake.State.FWD, 2),
              new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 2)
            ),
            new ParallelCommandGroup(
              new ToggleWristIn(wrist),
              new StoreCargo(upperIndex, lowerIndex)
            )
          )
        )
     );
  }
}
