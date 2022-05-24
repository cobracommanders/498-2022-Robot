package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleOpIntake extends CommandBase{
  private final Intake intake;
  private final DoubleSupplier speed;

  public TeleOpIntake(Intake intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(this.intake);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    intake.setSpeed(speed.getAsDouble()*.5);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}