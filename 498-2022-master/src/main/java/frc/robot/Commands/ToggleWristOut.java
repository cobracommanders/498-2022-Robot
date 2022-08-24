package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.Position;

public class ToggleWristOut extends CommandBase{
    private final Wrist wrist;
    public ToggleWristOut(Wrist wrist) {
      this.wrist = wrist;
      addRequirements(this.wrist);
    }
    @Override
    public void initialize(){
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      wrist.setPosition(Position.OUT);
      wrist.position = Position.OUT;
      //wrist.set(.2);
     // SmartDashboard.putNumber("WristPError", wrist.pidController.getPositionError());
    }
    @Override
    public boolean isFinished(){
      return wrist.getDownSensor()==true || wrist.pidController.atSetpoint();
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      wrist.stop();
    }
}