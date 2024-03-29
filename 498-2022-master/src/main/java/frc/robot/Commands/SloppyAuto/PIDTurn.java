package frc.robot.Commands.SloppyAuto;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDTurn extends CommandBase{
  private final Drivetrain drivetrain;
  private final double angle;
    double error, output, speed;
  public PIDTurn(Drivetrain drivetrain, double angle) {
    this.drivetrain = drivetrain;
    this.angle = angle;
    addRequirements(this.drivetrain);
  }
  public void initialize(){
    drivetrain.zeroHeading();
    drivetrain.setNeutralMode(NeutralMode.Brake);
    speed = -.20;
}
  // Called repeatedly when this Command is scheduled to run
  @Override
    public void execute() {
    if(angle<=0){
      output = -speed;
      error = angle-drivetrain.getHeading().getDegrees();
    }
    else{
      error = angle-drivetrain.getHeading().getDegrees();
      output = speed;
    }
    //output=.5;
    drivetrain.setPIDNoGyro(-output, output);
    //drivetrain.m_left_1.set(-output);
    //drivetrain.m_right_1.set(output);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(error)<=8;
    }
  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.m_left_1.set(0);
    drivetrain.m_right_1.set(0);    //m_drive = new DifferentialDrive(null, null);
  }
}