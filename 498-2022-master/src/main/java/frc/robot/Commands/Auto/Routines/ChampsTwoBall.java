package frc.robot.Commands.Auto.Routines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.ToggleIntake;
import frc.robot.Commands.ToggleLowerIndex;
import frc.robot.Commands.ToggleUpperIndex;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.Commands.SloppyAuto.DriveStraight;
import frc.robot.Commands.SloppyAuto.PIDTurn;
import frc.robot.Commands.SloppyAuto.TimedFlywheel;
import frc.robot.Commands.SloppyAuto.TimedShootCargo;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;

public class ChampsTwoBall extends SequentialCommandGroup{
    public ChampsTwoBall(DoubleSupplier shootStall, DoubleSupplier driveStall, DoubleSupplier spitStall, Blinkin blinkin, Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new ParallelDeadlineGroup(
                new DriveStraight(drivetrain, 1.3, false),
                new ToggleWristOut(wrist),
                new ToggleIntake(intake, Intake.State.FWD, wrist, blinkin),
                new ToggleUpperIndex(upperIndex, UpperIndex.State.REV),
                new ToggleLowerIndex(lowerIndex, LowerIndex.State.FWD),
                new TimedFlywheel(flywheel, Flywheel.State.REV, 3)
            ),
            new DriveStraight(drivetrain, 1.6, true),
            // new ParallelDeadlineGroup(
            //     new WaitCommand(1), 
            //     new ToggleIntake(intake, Intake.State.FWD, wrist, blinkin),
            //     new ToggleLowerIndex(lowerIndex, LowerIndex.State.FWD),
            //     new ToggleUpperIndex(upperIndex, UpperIndex.State.REV),
            //     new TimedFlywheel(flywheel, Flywheel.State.REV, 1)
            // ),
            new ParallelDeadlineGroup(
                new PIDTurn(drivetrain, -160),
                new ToggleWristIn(wrist)
            ),
            new ParallelDeadlineGroup(new WaitCommand(2),
                new DriveStraight(drivetrain, 1.5, false)),
            new DriveStraight(drivetrain, .5, true),
            new TimedFlywheel(flywheel, Flywheel.State.FULL, shootStall.getAsDouble()),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, .4),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 2)),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 1))
                //new WaitCommand(driveStall.getAsDouble())
            );
    }
}
