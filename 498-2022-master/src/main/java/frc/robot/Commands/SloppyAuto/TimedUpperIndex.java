package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.UpperIndex.State;

public class TimedUpperIndex extends CommandBase{
    private final UpperIndex UpperIndex;
    private final State state;
    private final double time;

    private Timer timer = new Timer();
    public TimedUpperIndex(UpperIndex UpperIndex, State state, double time) {
      this.UpperIndex = UpperIndex;
      this.state = state;
      this.time = time;
      addRequirements(this.UpperIndex);
    }
    @Override
    public void initialize() {
      timer.start();
      timer.reset();
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      UpperIndex.setState(state);
    }
    @Override
    public boolean isFinished(){
      return timer.get() >= time;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      UpperIndex.setState(State.OFF);
      timer.stop();
      timer.reset();
    }
}