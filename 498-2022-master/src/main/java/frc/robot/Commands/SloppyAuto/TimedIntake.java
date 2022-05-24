package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.State;

public class TimedIntake extends CommandBase{
    private final Intake intake;
    private final State state;
    private final double time;

    private Timer timer = new Timer();
    public TimedIntake(Intake intake, State state, double time) {
      this.intake = intake;
      this.state = state;
      this.time = time;
      addRequirements(this.intake);
    }
    @Override
    public void initialize() {
      timer.start();
      timer.reset();
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      intake.setState(state);
    }
    @Override
    public boolean isFinished(){
      return timer.get() >= time;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      intake.setState(State.OFF);
      timer.stop();
      timer.reset();
    }
}