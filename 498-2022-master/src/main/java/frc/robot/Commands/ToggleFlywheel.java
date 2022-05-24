package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Flywheel.State;

public class ToggleFlywheel extends CommandBase{
    private final Flywheel flywheel;
    private final State state;
    private final XboxController controller;
    private final XboxController controller1;

    public ToggleFlywheel(Flywheel flywheel, State state, XboxController controller, XboxController controller1) {
      this.flywheel = flywheel;
      this.state = state;
      this.controller = controller;
      this.controller1 = controller1;

      addRequirements(this.flywheel);
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      flywheel.setState(state);
      if(state != State.REV){
        controller.setRumble(RumbleType.kLeftRumble, .1);
      }
      if((flywheel.state == Flywheel.State.FWD && flywheel.getVelocity() >= 1700) || 
      (flywheel.state == Flywheel.State.FULL && flywheel.getVelocity() >= 3230)){
        controller1.setRumble(RumbleType.kLeftRumble, .1);
      }
      else{
        controller1.setRumble(RumbleType.kLeftRumble, 0);
      }
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      flywheel.setState(State.OFF);
      controller.setRumble(RumbleType.kLeftRumble, 0);
      controller1.setRumble(RumbleType.kLeftRumble, 0);

    }
}
