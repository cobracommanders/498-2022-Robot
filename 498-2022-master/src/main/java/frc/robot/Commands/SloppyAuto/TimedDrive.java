package frc.robot.Commands.SloppyAuto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TimedDrive extends CommandBase{
  private final Drivetrain drivetrain;
  private double distance;
  private double time;
  private Timer timer = new Timer();
  private final boolean reverse;
    double error, position, output = 0;
  public TimedDrive(Drivetrain drivetrain, double distance, boolean reverse, double time) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.reverse = reverse;
    this.time = time;
    addRequirements(this.drivetrain);
  }
  @Override
  public void initialize(){
      drivetrain.resetEncoders();
      drivetrain.setNeutralMode(NeutralMode.Brake);
      timer.start();
      timer.reset();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
      position = drivetrain.straightDistanceMeters();
      if(reverse){
        distance = -distance;
      }
        error =(distance-position)/distance;
          if(Math.abs(error)<=.6){
              output = .6;
          }
          else if(Math.abs(error)>=.7){
              output=.7;
          }
        else{
            output = error;
        }

        //output =.5/error;//DriveConstants.kPDriveVel.get()/ error;
        if(reverse){
          output = -output;
        }
        drivetrain.setPIDNoGyro(-output, -output);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(error)<=.1 || timer.get() >= time;
    }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    drivetrain.resetEncoders();
    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    //m_drive = new DifferentialDrive(null, null);
  }
}