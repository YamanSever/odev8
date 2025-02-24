// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  public static Swerve swerve = new Swerve();
  public static Limelight limelight = new Limelight();
  PS5Controller ps5 = new PS5Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  if (ps5.getRawButton(1)){
    double strafeValV = RobotContainer.swerve.visionVPIDController.calculate(LimelightHelpers.getTY(limelight),0);
    double strafeValX = RobotContainer.swerve.visionXPIDController.calculate(LimelightHelpers.getTY(limelight),0);
  }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
