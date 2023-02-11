// note: refactor the code

//main imports
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

// motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//joysticks
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
/*import java.sql.Time;
import java.sql.Timestamp;
import java.text.BreakIterator;
import java.util.concurrent.TimeUnit;*/
//import com.ctre.phoenix.time.StopWatch;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.*;
//import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController; 
// import com.ctre.phoenix.signals.*;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Robot extends TimedRobot {
  //Creating varibales for the motor controllers
  PWMVictorSPX driveLeftA = new PWMVictorSPX(1);
  PWMVictorSPX driveLeftB = new PWMVictorSPX(2);
  PWMVictorSPX driveRightA = new PWMVictorSPX(3);
  PWMVictorSPX driveRightB = new PWMVictorSPX(4);

  // variables for the arm controls
  CANSparkMax armYAxis = new CANSparkMax(11, MotorType.kBrushless);
  PWMVictorSPX armXAxis = new PWMVictorSPX(5);

  //variables for the pneumatics system
  Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  // joysticks
  Joystick driverController = new Joystick(1);
  XboxController armController = new XboxController(0);

  //current limit for the arm
  static final int ArmCurrentLimitA = 20;

  //Arm power output
  static final double ArmOutputPower = 0.1;

  //time to move the arm
  static final double ArmExtendTime = 2.0;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
   @Override

  //function for setting the initial conditions of all the hardware
  public void robotInit() {

    //initial conditions for the drive motors
    driveLeftA.setInverted(true);
    driveLeftB.setInverted(true);
    driveRightA.setInverted(false);
    driveRightB.setInverted(false);
    
    //initla conditions for the arm
    armYAxis.setInverted(true);
    armYAxis.setIdleMode(IdleMode.kBrake);
    armYAxis.setSmartCurrentLimit(ArmCurrentLimitA);
    ((CANSparkMax) armYAxis).burnFlash();
    armXAxis.setInverted(false);

    //initial conditions for the intake
    compressor.disable();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  /**
   * *set the arm output power. Positive is out, negative is in
   * 
   * @param percent
   */

    //function to set the arm output power in the vertical direction
  public void setArmYAxisMotor(double percent) {
    armYAxis.set(percent);
    SmartDashboard.putNumber("armYAxis power(%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", armYAxis.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature(C)", armYAxis.getMotorTemperature());
  }

  //function to set the arm output power in the horizontal direction
  public void setArmXAxisMotor(double percent) {
    armXAxis.set(percent);
    SmartDashboard.putNumber("armXaxis power(%)", percent);
    /*SmartDashboard.putNumber("armXAxis motor current (amps)", armXAxis.getVoltage());
    SmartDashboard.putNumber("armXAxis motor temperature(C)", armXAxis.getMotorTemperature());*/
  }
  
 /**
  * set the arm output power.
  *
  * @param percent desired speed
  * @param amps current limit
  */
  
  //function for starting autonomous
  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  //function that is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    for(int i = -1; i < 4; i += 2) {
      if (Timer.getFPGATimestamp() - autoStart > i) {
        driveLeftA.set(0.1);
        driveLeftB.set(0.1);
        driveRightA.set(0.1);
        driveRightB.set(0.1);
      }
      else if (Timer.getFPGATimestamp() - autoStart > i + 1) {
        driveLeftA.set(-0.4);
        driveLeftB.set(-0.4);
        driveRightA.set(0.4);
        driveRightB.set(0.4);
      } 
      else {
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }
  }
    
    /** else if (Timer.getFPGATimestamp() - autoStart > 3){
      driveLeftA.set(0.1);
      driveLeftB.set(0.1);
      driveRightA.set(0.1);
      driveRightB.set(0.1);
    } 
    else if (Timer.getFPGATimestamp() - autoStart > 4) {
      driveLeftA.set(-0.4);
      driveLeftB.set(-0.4);
      driveRightA.set(0.4);
      driveRightB.set(0.4);
    } 
    else if (Timer.getFPGATimestamp() - autoStart > 5) {
      driveLeftA.set(0.1);
      driveLeftB.set(0.1);
      driveRightA.set(0.1);
      driveRightB.set(0.1);
    }
    else if (Timer.getFPGATimestamp() - autoStart > 6) {
      driveLeftA.set()
    } */
    

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(4);
    
    double driveLeftPower = forward - turn;
    double driveRightPower = forward + turn;

    driveLeftA.set(driveLeftPower);
    driveLeftB.set(driveLeftPower);
    driveRightA.set(driveRightPower);
    driveRightB.set(driveRightPower);
    
    //Code for the arm
    double armPower;

    // motion for the arm in the vertical direction
    if (armController.getLeftY() > 0.5) {
      //raise the arm
      armPower = ArmOutputPower;
    }
    else if (armController.getLeftY() < -0.5) {
      //lower the arm
      armPower = -ArmOutputPower;
    }
    else {
      //do nothing and let it sit where it is
      armPower = 0.0;
      armYAxis.setIdleMode(IdleMode. kBrake);
    }
    setArmYAxisMotor(armPower);
    
    // motion for the arm in the horizontal direction
    if (armController.getLeftTriggerAxis() > 0.5) {
      //extend the arm
      armPower = ArmOutputPower;
    }
    else if (armController.getRightTriggerAxis() > 0.5) {
      //retract the arm
      armPower = -ArmOutputPower;
    }
    else {
      // do nothing and let it sit where it is
      armPower = 0.0;
      armXAxis.stopMotor();
    }
    setArmXAxisMotor(armPower);

    //Intake controls

    //solenoid controls
    if(armController.getLeftBumperPressed()){

      //fire the air one way
      solenoid.set(DoubleSolenoid.Value.kForward);
      
    } else if(armController.getRightBumperPressed()){

      //fire the air the other way
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //compressor controls
    if (armController.getAButton()) {

      //enable the compressdor
      compressor.enableAnalog(0, 50);

    } else if (armController.getBButton()) {

      //disable the compressor
      compressor.disable();
    }

   }

  //function for disabling everything at the end of the game
  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    armYAxis.set(0);
    armXAxis.set(0);
    compressor.disable();
  }
}