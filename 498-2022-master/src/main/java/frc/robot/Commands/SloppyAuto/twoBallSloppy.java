package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class twoBallSloppy extends SequentialCommandGroup{
    public twoBallSloppy(Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new ParallelCommandGroup(
                new ToggleWristOut(wrist),
                new TimedIntake(intake, Intake.State.FWD, 2.5),
                new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, 2.5),
                new DriveToStall(drivetrain, .5, .7),
                new TimedFlywheel(flywheel, Flywheel.State.REV, 2.5),
                new TimedUpperIndex(upperIndex, UpperIndex.State.REV, 2.5)),
                //new ToggleUpperIndex(upperIndex, UpperIndex.State.REV)),3
            new ParallelDeadlineGroup(
                new DriveToStall(drivetrain, -.5, .8),
                //new TimedTurn(drivetrain, 1.01),//swap for right positive left negative
                new ToggleWristIn(wrist)
                //new StoreCargo(upperIndex, flywheel, lowerIndex)3.8
            ),
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 1),//wait
            new ParallelCommandGroup(
                //new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
                new TimedTurn(drivetrain, 1.45)//swap for right positive left negative
                ),//9.3

            new ParallelCommandGroup(
                    new TimedFlywheel(flywheel, Flywheel.State.FULL, 3),
                    new DriveToStall(drivetrain, .4, 2)),//
            // new ParallelCommandGroup(
            //     new DriveToStall(drivetrain, -.5, .1),
            //     new TimedFlywheel(flywheel, Flywheel.State.FULL, .1)
            // ),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, .25),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2)),

            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 2),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2))
        );
    }
}
