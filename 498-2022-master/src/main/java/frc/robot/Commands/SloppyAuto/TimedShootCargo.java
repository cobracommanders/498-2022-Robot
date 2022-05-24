package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.UpperIndex.State;

public class TimedShootCargo extends CommandBase{
    private final UpperIndex upperIndex;
    private final LowerIndex lowerIndex;
    private final double time;

    Timer timer1 = new Timer();
    public TimedShootCargo(UpperIndex upperIndex, LowerIndex lowerIndex, double time) {
      this.upperIndex = upperIndex;
      this.lowerIndex = lowerIndex;
      this.time = time;
      addRequirements(this.upperIndex);
      addRequirements(this.lowerIndex);
    }
    @Override
  public void initialize(){
    timer1.start();
    timer1.reset();
  }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {      
      // if(flywheelVelocity.getAsBoolean()){ //calculated 880
      //   upperIndex.setState(UpperIndex.State.FWD);
      //   lowerIndex.setState(LowerIndex.State.FWD);
      // }
      // else{
      //   upperIndex.setState(UpperIndex.State.OFF);
      //   lowerIndex.setState(LowerIndex.State.OFF);
      //   end = true;
      // }
      upperIndex.setState(UpperIndex.State.FWD);
      lowerIndex.setState(LowerIndex.State.FWD);
    }
    @Override
    public boolean isFinished(){
        return timer1.get() >=time;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      upperIndex.setState(State.OFF);
      lowerIndex.setState(LowerIndex.State.OFF);
      timer1.stop();
      timer1.reset();
    }
}