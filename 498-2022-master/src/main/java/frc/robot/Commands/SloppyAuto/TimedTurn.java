package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TimedTurn extends CommandBase{
  private final Drivetrain drivetrain;
  private final double time;
  Timer timer = new Timer();
    double error, output, speed;
  public TimedTurn(Drivetrain drivetrain, double time) {
    this.drivetrain = drivetrain;
    this.time = time;
    addRequirements(this.drivetrain);
  }
  public void initialize(){
    drivetrain.zeroHeading();
    speed = .25;
    timer.start();
    timer.reset();
}
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
    output=speed;
    //output=.5;
    
    drivetrain.m_left_1.set(output);
    drivetrain.m_right_1.set(-output);
    }
    @Override
    public boolean isFinished(){
        return (timer.get() >= time);
    }
  @Override
  public void end(boolean interrupted) {
    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    //m_drive = new DifferentialDrive(null, null);
    timer.stop();
    timer.reset();
  }
}