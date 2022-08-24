package frc.robot.Commands.Auto.Routines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.SloppyAuto.DriveStraight;
import frc.robot.Commands.SloppyAuto.TimedFlywheel;
import frc.robot.Commands.SloppyAuto.TimedShootCargo;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class ChampsScoreandTaxi extends SequentialCommandGroup{
    public ChampsScoreandTaxi(DoubleSupplier shootStall, DoubleSupplier driveStall, Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new TimedFlywheel(flywheel, Flywheel.State.AUTO, shootStall.getAsDouble()),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, .5),
                new TimedFlywheel(flywheel, Flywheel.State.AUTO, .5)),
            new WaitCommand(driveStall.getAsDouble()),
            new DriveStraight(drivetrain, .75, true)
            );
    }
}
