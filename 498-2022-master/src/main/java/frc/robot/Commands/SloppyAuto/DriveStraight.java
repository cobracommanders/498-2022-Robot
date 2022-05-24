package frc.robot.Commands.SloppyAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase{
  private final Drivetrain drivetrain;
  private double distance;
  private final boolean reverse;
    double error, position, output = 0;
  public DriveStraight(Drivetrain drivetrain, double distance, boolean reverse) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.reverse = reverse;
    addRequirements(this.drivetrain);
  }
  @Override
  public void initialize(){
      drivetrain.resetEncoders();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
      position = drivetrain.straightDistanceMeters();
      if(reverse){
        distance = -distance;
      }
        error =(distance-position)/distance;
          if(error<=.5){
              output = .5;
          }
          else if(error>=.6){
              output=.6;
          }
        else{
            output = error;
        }

        //output =.5/error;//DriveConstants.kPDriveVel.get()/ error;
        if(reverse){
          output = -output;
        }
        drivetrain.m_left_1.set(output);
        drivetrain.m_right_1.set(output);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(error)<=.05;
    }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    //m_drive = new DifferentialDrive(null, null);
  }
}