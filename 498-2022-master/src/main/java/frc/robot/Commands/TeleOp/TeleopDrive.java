package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase{
  private final Drivetrain drivetrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;
  private double v0 = 0, v1 = 0, dv;
  private double r0 = 0, r1 = 0, dr;
  private DifferentialDrive m_drive;
  public TeleopDrive(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(this.drivetrain);
  }
  @Override
  public void initialize(){
    m_drive = new DifferentialDrive(drivetrain.m_left_1, drivetrain.m_right_1);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    v0 = speed.getAsDouble(); 
    r0 = rotation.getAsDouble();
    dv = v0-v1;
    dr = r0 - r1;
    
    if((Math.abs(dv) >= .03) || Math.abs(dr)<=.03){
      drivetrain.configRamp(.5);
    }else
    if(Math.abs(dv) >= .01|| Math.abs(dr)<=.01){
      drivetrain.configRamp(.3);
    }
    else 
    if(Math.abs(speed.getAsDouble())<=.05 && Math.abs(rotation.getAsDouble())<=.1){
      drivetrain.configRamp(0.9);
    }
    if(Math.abs(speed.getAsDouble())<=.05 && Math.abs(rotation.getAsDouble())>=.1){
      drivetrain.configRamp(0.1);
    }
    if(Math.abs(dr) >=.03){
      drivetrain.configRamp(.3);
    }
    v1 = v0;
    r1 = r0;
    m_drive.arcadeDrive(speed.getAsDouble()*.85, rotation.getAsDouble() * .65);
    SmartDashboard.putNumber("dv", dv);

    SmartDashboard.putNumber("Left Drive Velocity", drivetrain.leftWheelSpeedMetersPerSecond());
    SmartDashboard.putNumber("Right Drive Velocity", drivetrain.rightWheelSpeedMetersPerSecond());
    SmartDashboard.putNumber("Drive Heading", drivetrain.getHeading().getDegrees());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
