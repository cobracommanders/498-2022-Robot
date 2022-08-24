package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.State;

public class ToggleIntake extends CommandBase{
    private final Intake intake;
    private final State state;
    private final Wrist wrist;
    private final Blinkin blinkin;
    public ToggleIntake(Intake intake, State state, Wrist wrist, Blinkin blinkin) {
      this.intake = intake;
      this.state = state;
      this.wrist = wrist;
      this.blinkin = blinkin;
      addRequirements(this.intake);
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      if(wrist.getUpSensor() == true){
        intake.setState(State.OFF);
      }
      else{
        intake.setState(state);
      }
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      intake.setState(State.OFF);
    }
}