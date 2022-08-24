package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToStall extends CommandBase{
  private final Drivetrain drivetrain;
  private final double speed;
  private final double time;

    double error, output = 0;
  private Timer timer = new Timer();
  public DriveToStall(Drivetrain drivetrain, double speed, double time) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.time = time;
    addRequirements(this.drivetrain);
  }
  @Override
  public void initialize(){
      drivetrain.resetEncoders();
      timer.start();
      timer.reset();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
      drivetrain.m_left_1.set(-speed);
      drivetrain.m_right_1.set(-speed);
    }
    @Override
    public boolean isFinished(){
        return timer.get()>=time;//||drivetrain.stalling();
    }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    
    timer.stop();
    timer.reset();
    //m_drive = new DifferentialDrive(null, null);
  }
}