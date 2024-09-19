// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

   // Limelight NetworkTables
  private NetworkTable limelightTable;

   // Timer to control the duration of movement
  private Timer driveTimer = new Timer();

  // Store the AprilTag ID from SmartDashboard
  private int selectedAprilTagID;

  @Override
  public void robotInit() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    // Add an AprilTag ID selector to SmartDashboard
    SmartDashboard.putNumber("Target AprilTag ID", 0); // Default to 0 (no tag selected)
  }

  @Override
  public void autonomousInit() {
    // Reset the timer before starting autonomous
    driveTimer.reset();
    driveTimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
    
    // Get the selected AprilTag ID from SmartDashboard
    selectedAprilTagID = (int) SmartDashboard.getNumber("Target AprilTag ID", 0);

    // Fetch AprilTag ID from Limelight
    double[] aprilTagID = limelightTable.getEntry("tid").getDoubleArray(new double[0]);

    // If the Limelight sees the selected AprilTag ID, move forward for 1 second
    if (aprilTagID.length > 0 && aprilTagID[0] == selectedAprilTagID) {
      if (driveTimer.get() <= 1.0) {  // Drive for one second
        driveForward(0.2);  // Drive forward at 0.2 speed
      } else {
        m_swerve.stop();  // Stop after 1 second
      }
    } else {
      m_swerve.stop();  // Stop if no tag detected or not the right tag
    }

    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
