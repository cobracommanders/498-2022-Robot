package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.LowerIndex.State;

public class TimedLowerIndex extends CommandBase{
    private final LowerIndex LowerIndex;
    private final State state;
    private final double time;

    private Timer timer = new Timer();
    public TimedLowerIndex(LowerIndex LowerIndex, State state, double time) {
      this.LowerIndex = LowerIndex;
      this.state = state;
      this.time = time;
      addRequirements(this.LowerIndex);
    }
    @Override
    public void initialize() {
      timer.start();
      timer.reset();
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      LowerIndex.setState(state);
    }
    @Override
    public boolean isFinished(){
      return timer.get() >= time;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      LowerIndex.setState(State.OFF);
      timer.stop();
      timer.reset();
    }
}