package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.UpperIndex.State;

public class ToggleUpperIndex extends CommandBase{
    private final UpperIndex UpperIndex;
    private final State state;
    public ToggleUpperIndex(UpperIndex UpperIndex, State state) {
      this.UpperIndex = UpperIndex;
      this.state = state;
      addRequirements(this.UpperIndex);
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      UpperIndex.setState(state);
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      UpperIndex.setState(State.OFF);;
    }
}