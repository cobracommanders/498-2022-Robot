package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperIndex;

public class TeleOpUpperIndex extends CommandBase{
  private final UpperIndex upperIndex;
  private final DoubleSupplier speed;

  public TeleOpUpperIndex(UpperIndex upperIndex, DoubleSupplier speed) {
    this.upperIndex = upperIndex;
    this.speed = speed;
    addRequirements(this.upperIndex);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    upperIndex.setSpeed(speed.getAsDouble());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    upperIndex.stop();
  }
}