package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.Position;

public class ToggleClimber extends CommandBase{
    private final Climber climber;
    private final Position position;
    double error = 3;
    boolean l_finished = false;
    boolean r_finished = false;

    public ToggleClimber(Climber climber,Position position) {
      this.climber = climber;
      this.position = position;
      addRequirements(this.climber);
    }
    @Override
    public void initialize(){
    }
    // Called repeatedly when this Command is scheuled to run
    @Override
    public void execute() {
      //Climber.setPosition(Position.LOW);
       if(climber.m_leftEncoder.getPosition()<= position.setPoint-error){
         climber.setSpeed(.95);
       }
       else if(climber.m_leftEncoder.getPosition()>= position.setPoint+error){
         climber.setSpeed(-.95);
       }
      //  if(climber.rightEncoderValue() >= climber.m_leftEncoder.getPosition()+error){
      //   climber.m_right.set(-1);
      //  }
      //  else if(climber.rightEncoderValue() <= climber.m_leftEncoder.getPosition()-error){
      //    climber.m_right.set(1);
      //  }
      //Climber.set(.2);
    }
    @Override
    public boolean isFinished(){
      return Math.abs(position.setPoint - climber.m_leftEncoder.getPosition())<=error;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      climber.stop();
    }
}