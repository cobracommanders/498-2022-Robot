package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class TestWrist extends CommandBase{
  private final Wrist Wrist;
  private final DoubleSupplier speed;

  public TestWrist(Wrist Wrist, DoubleSupplier speed) {
    this.Wrist = Wrist;
    this.speed = speed;
    addRequirements(this.Wrist);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Wrist.set(speed.getAsDouble());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Wrist.stop();
  }
}