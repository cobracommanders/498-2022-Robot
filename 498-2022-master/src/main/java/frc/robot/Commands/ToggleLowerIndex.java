package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.LowerIndex.State;

public class ToggleLowerIndex extends CommandBase{
    private final LowerIndex LowerIndex;
    private final State state;
    public ToggleLowerIndex(LowerIndex LowerIndex, State state) {
      this.LowerIndex = LowerIndex;
      this.state = state;
      addRequirements(this.LowerIndex);
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      LowerIndex.setState(state);
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      LowerIndex.setState(State.OFF);;
    }
}