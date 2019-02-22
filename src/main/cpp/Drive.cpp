#include "Drive.hpp"

#include <Robot.h>

DriveManager::DriveManager () {
    stick = new frc::Joystick{ 0 };

    //driveMotorFrontLeft = new WPI_TalonSRX(1);
    //driveMotorFrontRight = new WPI_TalonSRX(3);
    //driveMotorBackLeft = new WPI_TalonSRX(2);
    //driveMotorBackRight = new WPI_TalonSRX(4);

    driveMotorFrontLeft = new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);
    driveMotorFrontRight = new rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);
    driveMotorBackLeft = new rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);
    driveMotorBackRight = new rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);


    //encoders for CANSparkMax
    encFrontLeft = new rev::CANEncoder(*driveMotorFrontLeft);
    encFrontRight = new rev::CANEncoder(*driveMotorFrontRight);
    encBackLeft = new rev::CANEncoder(*driveMotorBackLeft);
    encBackRight = new rev::CANEncoder(*driveMotorBackRight);

    mecanumDrive = new frc::MecanumDrive(*driveMotorFrontLeft, *driveMotorBackLeft, *driveMotorFrontRight, *driveMotorBackRight);

    //frontLeftPID = new frc::PIDController(0.0, 0.0, 0.0, &encFrontLeft, &driveMotorFrontLeft);

    //Gyro
    try {
        ahrs = new AHRS(SPI::Port::kMXP);
    }
    catch(std::exception ex) {
        std::string err_string = "Error initalizing naxX-MXP"; 
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
    ahrs->Reset(); 

    time = new frc::Timer;

    //Joystick values
    xStickValue = new double; 
    yStickValue = new double; 
    zStickValue = new double; 
    
    driveGyro = new double;
    gyro = new double; 
    error = new double;

    p = new double;
    i = new double; 
    integral = new double;
    d = new double; 
    prevError = new double;

    *p = 0.0095;
    *i = 0;
    *d = 0;
    *integral = 0;
    *prevError = 0;

    driveToggle = new bool;  
    driveLatch = new bool; 
    *driveToggle = true;
    *driveLatch = false;

    idleModeToggle = new bool;
    idleModeLatch = new bool;
    *idleModeToggle = false;  //true for brake
    *idleModeLatch = false; 

    
}

void DriveManager::driveTrain() {

    //*xStickValue = -stick->GetRawAxis(0);
    //*yStickValue = -stick->GetRawAxis(1);
    //*zStickValue = stick->GetRawAxis(2);


        if (abs(stick->GetRawAxis(1)) < .2) {
			*xStickValue = 0;
		}
		else {
			*xStickValue = -stick->GetRawAxis(1);
		}

		//Repeat of above for Y
		if (-stick->GetRawAxis(0) < -0.1 and -stick->GetRawAxis(0) > 0.4)
		{
			*yStickValue = 0;
		}
		else
		{
			*yStickValue = stick->GetRawAxis(0);
		}

		//Repeat of above for Z
		if (abs(stick->GetRawAxis(2)) < .1)
		{
			*zStickValue = 0;
		}
		else
		{
			*zStickValue = stick->GetRawAxis(2);
		}

        if (stick->GetRawButton(1)) {
            *xStickValue = *xStickValue * 0.35;
            *yStickValue = *yStickValue * 0.35;
            *zStickValue = *zStickValue * 0.35;
        }

    frc::SmartDashboard::PutNumber("joystickY", stick->GetRawAxis(0));
    frc::SmartDashboard::PutNumber("joystickx", stick->GetRawAxis(1));


    if (stick->GetRawButton(6) and !*driveLatch) {
        *driveToggle = !*driveToggle;
        *driveLatch = true;
    }
    else if (!stick->GetRawButton(6) and *driveLatch) {
        *driveLatch = false;
    }

    if (*driveToggle) {
        *driveGyro = 0;
    }
    else if (!*driveToggle) {
        *driveGyro = ahrs->GetAngle();
    }

    frc::SmartDashboard::PutNumber("driveGyro", *driveGyro);

    mecanumDrive->DriveCartesian(*yStickValue, *xStickValue, *zStickValue, *driveGyro);

    *gyro = ahrs->GetAngle(); 
    frc::SmartDashboard::PutNumber("gyro", *gyro);

    if (stick->GetRawButton(4)) {
        ahrs->Reset();
    }

    //velocities for CANSparkMax
    frc::SmartDashboard::PutNumber("velocityFrontLeft", encFrontLeft->GetVelocity());
    frc::SmartDashboard::PutNumber("velocityFrontRight", encFrontRight->GetVelocity());
    frc::SmartDashboard::PutNumber("velocityFBackLeft", encBackLeft->GetVelocity());
    frc::SmartDashboard::PutNumber("velocityBackRight", encBackRight->GetVelocity());

    //motor temp for CANSparkMax only
    frc::SmartDashboard::PutNumber("tempFrontLeft", driveMotorFrontLeft->GetMotorTemperature());
    frc::SmartDashboard::PutNumber("tempFrontRight", driveMotorFrontRight->GetMotorTemperature());
    frc::SmartDashboard::PutNumber("tempBackLeft", driveMotorBackLeft->GetMotorTemperature());
    frc::SmartDashboard::PutNumber("tempBackRight", driveMotorBackRight->GetMotorTemperature());

    //current and voltage
    frc::SmartDashboard::PutNumber("currentFrontLeft", driveMotorFrontLeft->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("currentFrontRight", driveMotorFrontRight->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("currentBackLeft", driveMotorBackLeft->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("currentBackRight", driveMotorBackRight->GetOutputCurrent());

    frc::SmartDashboard::PutNumber("voltageFrontLeft", driveMotorFrontLeft->GetBusVoltage());
    frc::SmartDashboard::PutNumber("voltageFrontRight", driveMotorFrontRight->GetBusVoltage());
    frc::SmartDashboard::PutNumber("voltageBackLeft", driveMotorBackLeft->GetBusVoltage());
    frc::SmartDashboard::PutNumber("voltageBackRight", driveMotorBackRight->GetBusVoltage());

    if (stick->GetRawButton(11) and !*idleModeLatch) {
        *idleModeToggle = !*idleModeToggle;
        *idleModeLatch = true;
    }
    else if (!stick->GetRawButton(11) and *idleModeLatch) {
        *idleModeLatch = false;
    }

    if (*idleModeToggle) {
        driveMotorFrontLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorFrontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorBackLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorBackRight->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        frc::SmartDashboard::PutString("driveMotorIdleMode", "brake");
    }
    else if (!*idleModeToggle) {
        driveMotorFrontLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        driveMotorFrontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        driveMotorBackLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        driveMotorBackRight->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        frc::SmartDashboard::PutString("driveMotorIdleMode", "coast");
    }
    
}

void DriveManager::control(double turn, double strafe, double drive, bool brake) { 
    mecanumDrive->DriveCartesian(strafe, drive, turn, 0);

    if (brake) {
        driveMotorFrontLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorFrontRight->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorBackLeft->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        driveMotorBackRight->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        frc::SmartDashboard::PutString("driveMotorIdleMode", "brake");
    }

}

void DriveManager::turn(int angle) {
    double power; 
    double turnP = 0.0095;
   // int want = angle;

    *gyro = ahrs->GetAngle();
    frc::SmartDashboard::PutNumber("gyro", *gyro);

  //  power = (-(*gyro - angle) * turnP);

 //   if (fabs(*gyro - angle) < 1) {
 //       power = 0;
 //   }

    *error = -(*gyro - angle);
    *integral += *error;
    power = (*p * *error) + (*i * *integral) + (*d * *prevError);
    *prevError = *error;

    frc::SmartDashboard::PutNumber("auto power", power);
    mecanumDrive->DriveCartesian(0, 0, power, 0);
}

void DriveManager::reset() {
    ahrs->Reset();
    //*time->Reset();

    *integral = 0;
    *prevError = 0;
}