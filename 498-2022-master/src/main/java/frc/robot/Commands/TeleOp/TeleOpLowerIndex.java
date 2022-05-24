package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerIndex;

public class TeleOpLowerIndex extends CommandBase{
  private final LowerIndex lowerIndex;
  private final DoubleSupplier speed;

  public TeleOpLowerIndex(LowerIndex lowerIndex, DoubleSupplier speed) {
    this.lowerIndex = lowerIndex;
    this.speed = speed;
    addRequirements(this.lowerIndex);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    lowerIndex.setSpeed(speed.getAsDouble());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    lowerIndex.stop();
  }
}