package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TestClimber extends CommandBase{
  private final Climber Climber;
  private final DoubleSupplier speed;

  public TestClimber(Climber Climber, DoubleSupplier speed) {
    this.Climber = Climber;
    this.speed = speed;
    addRequirements(this.Climber);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if(Math.abs(speed.getAsDouble())>=.10){
    Climber.setSpeed(speed.getAsDouble());
    }
    else{
      Climber.stop();
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Climber.stop();
  }
}