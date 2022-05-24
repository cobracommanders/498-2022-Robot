package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleRumble extends CommandBase{
    private final XboxController controller;
    private final double strength;
    public ToggleRumble(XboxController controller, double strength) {
      this.strength = strength;
      this.controller = controller;
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      controller.setRumble(RumbleType.kLeftRumble, strength);
    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      controller.setRumble(RumbleType.kLeftRumble, 0);

    }
}
