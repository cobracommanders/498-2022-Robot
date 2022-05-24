package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.State;

public class TimedFlywheel extends CommandBase{
    private final Flywheel Flywheel;
    private final State state;
    private final double time;

    private Timer timer = new Timer();
    public TimedFlywheel(Flywheel Flywheel, State state, double time) {
      this.Flywheel = Flywheel;
      this.state = state;
      this.time = time;
      addRequirements(this.Flywheel);
    }
    @Override
    public void initialize() {
      timer.start();
      timer.reset();
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      Flywheel.setState(state);
    }
    @Override
    public boolean isFinished(){
      return timer.get() >= time;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      Flywheel.setState(State.OFF);
      timer.stop();
      timer.reset();
    }
}