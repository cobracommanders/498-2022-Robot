package frc.robot.Commands.SloppyAuto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
      drivetrain.setNeutralMode(NeutralMode.Brake);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
      position = drivetrain.straightDistanceMeters();
      if(reverse){
        distance = -distance;
      }
      else{
        distance = -distance;
      }
      error =(distance-position)/distance;

          if(Math.abs(error)<=.5){
              output = .4;
          }
          else if(Math.abs(error)>=.6){
              output=.4;
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
        return Math.abs(error)<=.1;
    }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.resetEncoders();
    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    //m_drive = new DifferentialDrive(null, null);
  }
}