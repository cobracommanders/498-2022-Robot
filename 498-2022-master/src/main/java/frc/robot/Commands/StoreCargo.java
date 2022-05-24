package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;

public class StoreCargo extends CommandBase{
  private Timer timer = new Timer();
    private final UpperIndex upperIndex;
    private final LowerIndex lowerIndex;

    public StoreCargo(UpperIndex upperIndex, LowerIndex lowerIndex) {
      //this.lowerIndex = lowerIndex;
      this.upperIndex = upperIndex;
      this.lowerIndex = lowerIndex;
      //addRequirements(this.lowerIndex);
      addRequirements(this.upperIndex);
      addRequirements(this.lowerIndex);
    }
    @Override
    public void initialize(){
      timer.start();
      timer.reset();
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      //lowerIndex.setSpeed(-.8);
      upperIndex.setState(UpperIndex.State.REV);;
      lowerIndex.setState(LowerIndex.State.REV);;
    }
    @Override
    public boolean isFinished(){
      return timer.get()>= .25;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      //lowerIndex.stop();

      upperIndex.setState(UpperIndex.State.OFF);;
      lowerIndex.setState(LowerIndex.State.OFF);;
    }
}