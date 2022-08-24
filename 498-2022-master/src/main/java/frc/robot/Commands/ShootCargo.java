package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.UpperIndex.State;

public class ShootCargo extends CommandBase{
    private final UpperIndex upperIndex;
    private final LowerIndex lowerIndex;
    private final Flywheel flywheel;
    private final XboxController controller;
    boolean end = false;
    Timer timer1 = new Timer();
    public ShootCargo(UpperIndex upperIndex, LowerIndex lowerIndex, Flywheel flywheel, XboxController controller) {
      this.upperIndex = upperIndex;
      this.lowerIndex = lowerIndex;
      this.flywheel = flywheel;
      this.controller = controller;
      addRequirements(this.upperIndex);
      addRequirements(this.lowerIndex);
    }
    @Override
  public void initialize(){
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
      if((flywheel.state == Flywheel.State.FWD && Math.abs(flywheel.getVelocity()-2000) <= 3000) || //1900//1700
      (flywheel.state == Flywheel.State.FULL && Math.abs(flywheel.getVelocity()-3650) <= 3000)){//3220//3230
        upperIndex.setState(UpperIndex.State.FWD);
        lowerIndex.setState(LowerIndex.State.FWD);
        controller.setRumble(RumbleType.kLeftRumble, 0.1);
      }
      else{
        upperIndex.setState(State.OFF);
        lowerIndex.setState(LowerIndex.State.OFF);
        controller.setRumble(RumbleType.kLeftRumble, 0);
      }
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      controller.setRumble(RumbleType.kLeftRumble, 0);
      upperIndex.setState(State.OFF);
      lowerIndex.setState(LowerIndex.State.OFF);
    }
}