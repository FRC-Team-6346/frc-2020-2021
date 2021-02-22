/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
// import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.util.Color;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // For driving
  private XboxController xbox = new XboxController(0);
  // For shooting
  private XboxController xbox2 = new XboxController(1);
  // private DoubleSolenoid solenoid = new DoubleSolenoid(4, 7);
  // private Compressor compressor = new Compressor();
  // Drive
  private WPI_VictorSPX spx12 = new WPI_VictorSPX(0);
  private WPI_VictorSPX spx13 = new WPI_VictorSPX(1);
  private WPI_VictorSPX spx14 = new WPI_VictorSPX(2);
  private WPI_VictorSPX spx15 = new WPI_VictorSPX(3);
  private SpeedControllerGroup right = new SpeedControllerGroup(spx12, spx13);
  private SpeedControllerGroup left = new SpeedControllerGroup(spx14, spx15);
  private DifferentialDrive tank = new DifferentialDrive(right, left);
  
  private DigitalInput switch1 = new DigitalInput(0);
  private DigitalInput switch2 = new DigitalInput(1);
  
  private Spark shoot1 = new Spark(7);
  private Spark shoot2 = new Spark(6);
  private SpeedControllerGroup shoot = new SpeedControllerGroup(shoot1, shoot2);
  
  private Spark lifter = new Spark(4);
  private Spark turret = new Spark(0);
  private Spark intake = new Spark(2);
  private Spark shootFeeder = new Spark(1);

  // Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private final Timer m_timer = new Timer();

  // Color sensor
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();
  // private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  // private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  // private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  // private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Color Sensing
    // m_colorMatcher.addColorMatch(kBlueTarget);
    // m_colorMatcher.addColorMatch(kGreenTarget);
    // m_colorMatcher.addColorMatch(kRedTarget);
    // m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_timer.start();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() < 0.5) {
      tank.arcadeDrive(0, 0.7);
    } else {
      tank.stopMotor();
    }
    if (m_timer.get() < 4.0) {
      // Run limelight
      double x = tx.getDouble(0.0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      double headingError = x;
      double Kp = -0.1;
      double minCommand = 0.05;
      double steeringAdjust = 0.0;
      if (x > 1.0) {
        steeringAdjust = Kp * headingError - minCommand;
      } else if (headingError < 1.0) {
        steeringAdjust = Kp * headingError + minCommand;
      }
      if (steeringAdjust > 0 && switch2.get()) {
        steeringAdjust = 0;
      }
      if (steeringAdjust < 0 && switch1.get()) {
        steeringAdjust = 0;
      }
      turret.set(steeringAdjust);
    } else {
      // Turn off light
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
    if (m_timer.get() > 6.0) {
      shootFeeder.set(-1);
      lifter.set(0.4);
    } else {
      shootFeeder.set(0);
      lifter.set(0);
    }
    if (m_timer.get() > 3.0 && m_timer.get() < 20.0) {
      shoot.set(-1);
    }
    else {
      shoot.set(0);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Drive
    double speed = xbox.getRawAxis(0);
    // deadband
    if (speed < 0.1 && speed > -0.1) {
      speed = 0.0;
    }
    tank.arcadeDrive(-speed * 0.75, xbox.getRawAxis(1) * 0.75);

    // Lifter
    double lifterspeed = xbox2.getRawAxis(1);
    // deadband
    if (lifterspeed < 0.1 && lifterspeed > -0.1) {
      lifterspeed = 0.0;
    }
    lifter.setSpeed(-lifterspeed);

    // Limelight
    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // if (xbox.getYButton()) {
    //   double Kp = -0.07;
    //   double steering_adjust = Kp * x;
    //   tank.tankDrive(-steering_adjust, steering_adjust);
    // } else {
    //   tank.arcadeDrive(speed, xbox.getRawAxis(0));
    // }

    // Turret Aiming
    if (xbox2.getYButton()) {   
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      double headingError = x;
      double Kp = -0.1;
      double minCommand = 0.05;
      double steeringAdjust = 0.0;
      if (x > 1.0) {
        steeringAdjust = Kp * headingError - minCommand;
      } else if (headingError < 1.0) {
        steeringAdjust = Kp * headingError + minCommand;
      }
      if (steeringAdjust > 0 && switch2.get()) {
        steeringAdjust = 0;
      }
      if (steeringAdjust < 0 && switch1.get()) {
        steeringAdjust = 0;
      }
      turret.set(steeringAdjust);
    } else {
      // Turn off light
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      // Turret movement
      double turretSpeed = 0;
      if (xbox2.getBumper(Hand.kLeft)) {
        turretSpeed = -1;
      }
      if (xbox2.getBumper(Hand.kRight)) {
        turretSpeed = 1;
      }
      if (turretSpeed > 0 && switch1.get()) {
        turretSpeed = 0;
      }
      if (turretSpeed < 0 && switch2.get()) {
        turretSpeed = 0;
      }
      turret.set(-turretSpeed);
    }   
    
    // Intake
    double intakeSpeed = xbox2.getTriggerAxis(Hand.kRight);
    // deadband
    if (intakeSpeed < 0.1 && intakeSpeed > -0.1) {
      intakeSpeed = 0.0;
    }
    if (xbox2.getBButton()) {
      intakeSpeed = -1;
    }
    intake.set(-intakeSpeed);

    // Color sensing
    // Color detectedColor = m_colorSensor.getColor();
    // String colorString;
    // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // if (match.color == kBlueTarget) {
    //   colorString = "Blue";
    // } else if (match.color == kRedTarget) {
    //   colorString = "Red";
    // } else if (match.color == kGreenTarget) {
    //   colorString = "Green";
    // } else if (match.color == kYellowTarget) {
    //   colorString = "Yellow";
    // } else {
    //   colorString = "Unknown";
    // }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", colorString);

    // Shooter
    double shootSpeed = xbox2.getTriggerAxis(Hand.kLeft);
    shoot.set(-shootSpeed);

    // Shootfeeder
    double feederSpeed = 0;
    if (xbox2.getXButton()) {
      feederSpeed = 1;
    }
    shootFeeder.set(-feederSpeed);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
