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

public class FenderSloppyHigh extends SequentialCommandGroup{
    public FenderSloppyHigh(Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new ParallelCommandGroup(
                new ToggleWristOut(wrist),
                new IntakeDrive(drivetrain, intake, lowerIndex, 2.5, Intake.State.FWD, 3, false)),
            new ParallelCommandGroup(
                new DriveTurn(drivetrain, 173),
                new ToggleWristIn(wrist),
                new StoreCargo(upperIndex, lowerIndex)
            ),
            new StoreCargo(upperIndex, lowerIndex),
            new ParallelCommandGroup(
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 4),
                new DriveToStall(drivetrain, .5, 3.5)
            ),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, .4),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2)),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2))
        );
    }
}
