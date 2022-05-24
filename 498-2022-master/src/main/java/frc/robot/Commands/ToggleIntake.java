package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.State;

public class ToggleIntake extends CommandBase{
    private final Intake intake;
    private final State state;
    public ToggleIntake(Intake intake, State state) {
      this.intake = intake;
      this.state = state;
      addRequirements(this.intake);
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      intake.setState(state);
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      intake.setState(State.OFF);;
    }
}