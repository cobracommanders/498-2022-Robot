package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Blinkin.Color;

public class BlinkinCommand extends CommandBase{
  private final Blinkin data;
    public BlinkinCommand(Blinkin data) {
      this.data = data;
      this.addRequirements(this.data);
    }
  
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
      if(data.getSensorData(data.m_topStorageSensor) && !data.getSensorData(data.m_bottomStorageSensor)){
        data.setColor(Color.BLUE);
      }
      else if(data.getSensorData(data.m_topStorageSensor) && data.getSensorData(data.m_bottomStorageSensor)){
        data.setColor(Color.GREEN);
      }
      else{
        data.setColor(Color.WHITE);
      }

    }
    @Override
    public boolean isFinished(){
      return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}
