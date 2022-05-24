package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.StoreCargo;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class ThreeBallSloppy extends SequentialCommandGroup{
    public ThreeBallSloppy(Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new ParallelCommandGroup(
                new ToggleWristOut(wrist),
                new IntakeDrive(drivetrain, intake, lowerIndex, 1.25, Intake.State.FWD, 2, false)),
            new ParallelCommandGroup(
                new DriveTurn(drivetrain, 180),
                new ToggleWristIn(wrist),
                new StoreCargo(upperIndex, lowerIndex)
            ),
            new StoreCargo(upperIndex, lowerIndex),
            new ParallelCommandGroup(
                new TimedFlywheel(flywheel, Flywheel.State.FWD, 4),
                new DriveToStall(drivetrain, .5, 3.5)
            ),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, .2),
                new TimedFlywheel(flywheel, Flywheel.State.FWD, 1)),
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2)),
            new DriveStraight(drivetrain, 1, true),
            new DriveTurn(drivetrain, 90),
            new ParallelCommandGroup(
                new ToggleWristOut(wrist),
                new IntakeDrive(drivetrain, intake, lowerIndex, 2.5, Intake.State.FWD, 3, false)),
            new ParallelCommandGroup(
                new TimedFlywheel(flywheel, Flywheel.State.AUTO, 2.5),
                new DriveTurn(drivetrain, -150),
                new ToggleWristIn(wrist)
            ),
            new ParallelCommandGroup(
                new TimedFlywheel(flywheel, Flywheel.State.AUTO, 3),
                new DriveStraight(drivetrain, 2, false)
            ),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.AUTO, 2))
        );
    }
}
