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

public class ChampsRight extends SequentialCommandGroup{
    public ChampsRight(DoubleSupplier shootStall, DoubleSupplier driveStall, DoubleSupplier spitStall, Blinkin blinkin, Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, UpperIndex upperIndex, Flywheel flywheel, Wrist wrist){
        super(
            new TimedFlywheel(flywheel, Flywheel.State.FULL, shootStall.getAsDouble()),
            new ParallelCommandGroup(
                new TimedShootCargo(upperIndex, lowerIndex, 1),
                new TimedFlywheel(flywheel, Flywheel.State.FULL, 1)
            ),
            new WaitCommand(driveStall.getAsDouble()),
            new DriveStraight(drivetrain, .25, true),
            new PIDTurn(drivetrain, -115),
            new ParallelDeadlineGroup(
                new DriveStraight(drivetrain, 1.8, false),
                new ToggleWristOut(wrist),
                new ToggleIntake(intake, Intake.State.FWD, wrist, blinkin)
            ),
            new ParallelDeadlineGroup(
                new PIDTurn(drivetrain, -110),//.andThen(new TimedDrive(drivetrain, 1, false, .1)),
                // new SequentialCommandGroup(
                //     new ParallelDeadlineGroup(new WaitCommand(1), new ToggleWristIn(wrist)),
                //     new ParallelDeadlineGroup(new WaitCommand(1), new ToggleWristOut(wrist))
                // ),
                new ToggleUpperIndex(upperIndex, UpperIndex.State.REV),
                new ToggleLowerIndex(lowerIndex, LowerIndex.State.FWD),
                new ToggleIntake(intake, Intake.State.FWD, wrist, blinkin)
            ),
            new WaitCommand(spitStall.getAsDouble()),
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new ToggleIntake(intake, Intake.State.SPIT, wrist, blinkin),
                new ToggleUpperIndex(upperIndex, UpperIndex.State.REV),
                new ToggleLowerIndex(lowerIndex, LowerIndex.State.REV)
            ),
            //new DriveStraight(drivetrain, .2, true),
            new ParallelDeadlineGroup(
                new PIDTurn(drivetrain, -180),
                new ToggleWristIn(wrist)
            )
            // new ParallelCommandGroup(
            //     new TimedShootCargo(upperIndex, lowerIndex, 1),
            //     new TimedFlywheel(flywheel, Flywheel.State.FWD, 2)),
            );
    }
}
