package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;

public class IntakeDrive extends ParallelCommandGroup{
    IntakeDrive(Drivetrain drivetrain, Intake intake, LowerIndex lowerIndex, double distance, Intake.State state, double intakeTime, boolean reverse){
        super(
            new DriveStraight(drivetrain, distance, reverse),
            new TimedIntake(intake, state, intakeTime),
            new TimedLowerIndex(lowerIndex, LowerIndex.State.FWD, intakeTime)
        );
    }
}
