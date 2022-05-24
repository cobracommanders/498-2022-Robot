package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class ScorethenTaxi extends SequentialCommandGroup{
    public ScorethenTaxi(Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new TimedFlywheel(flywheel, Flywheel.State.FULL, 2),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2)),
            // new ParallelCommandGroup(
            //     new TimedShootCargo(upperIndex, lowerIndex, 1),
            //     new TimedFlywheel(flywheel, Flywheel.State.FWD, 2)),
            new WaitCommand(6),
            new DriveStraight(drivetrain, 2.5, true)
            );
    }
}
