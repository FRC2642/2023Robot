/* The 2023 Prayer
 * 
 * To Kyle, whose code will never be perfect enough and will always be rewritten
 * To Thomas, and the uh-oh button that never was
 * To Kaiji, who worked on complex image processing code that came up a little short compared to a $400 limelight
 * To Alec, thank you for rewriting Black Pearl for us to use
 * 
 * 
 * To Arianna, who left to join the scouting team
 * To Saline, who left to join the engineering team
 * To Hadleigh, who left to join the design team
 * 
 * To everyone else, whose code was rewritten a hundred times over
 * 
 * RIP Thin Mints, which were only eaten once
 * RIP Uh-oh button, which was so close to becoming a reality
 * RIP Joonoh's Claw prototypes 1-58, 59 was better
 * 
 * Thank you limelight, for being a gamechanger for us while also being a pain in the butt
 * Thank you swerve modules, for not having to have wheel watchers to work correctly
 * Thank you path planner, for allowing us to create a 34 cube auto 
 * 
 * And finally, to robot.go(), which was not accepted by everyone and was deleted forever
 * 
 */



















package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    RobotContainer.DEBUG = (int)SmartDashboard.getNumber("DEBUG MODE", 0) != 0;
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    

    SmartDashboard.putBoolean("RED", DriverStation.getAlliance() == Alliance.Red);
    SmartDashboard.putBoolean("BLUE", DriverStation.getAlliance() == Alliance.Blue);

    if (m_robotContainer.shoulder.getCurrentCommand() != null && Math.abs(m_robotContainer.auxControl.getLeftY()) > 0.1 && m_robotContainer.shoulder.getCurrentCommand().getName() != m_robotContainer.shoulder.getDefaultCommand().getName()) m_robotContainer.shoulder.getCurrentCommand().cancel();
    if (m_robotContainer.wrist.getCurrentCommand() != null && (m_robotContainer.auxControl.getXButton() == true || m_robotContainer.auxControl.getYButton() == true) && m_robotContainer.wrist.getCurrentCommand().getName() != m_robotContainer.wrist.getDefaultCommand().getName()) m_robotContainer.wrist.getCurrentCommand().cancel();
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
   // CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

