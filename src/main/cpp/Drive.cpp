#include "Drive.hpp"

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

    //Joystick values
    xStickValue = new double; 
    yStickValue = new double; 
    zStickValue = new double; 
    
    driveGyro = new double;
    gyro = new double; 

    driveToggle = new bool;  
    driveLatch = new bool; 
    *driveToggle = true;
    *driveLatch = false;
}

void DriveManager::driveTrain() {
    //*xStickValue = -stick->GetRawAxis(0);
    //*yStickValue = -stick->GetRawAxis(1);
    //*zStickValue = stick->GetRawAxis(2);


    if (abs(stick->GetRawAxis(1)) < .2)
		{
			*xStickValue = 0;
		}
		//Otherwise set to joystick value
		else
		{
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

    frc::SmartDashboard::PutNumber("joystickY", stick->GetRawAxis(1));
    frc::SmartDashboard::PutNumber("joystickx", stick->GetRawAxis(1));


    if (stick->GetRawButton(3) and !*driveLatch) {
        *driveToggle = !*driveToggle;
        *driveLatch = true;
    }
    else if (!stick->GetRawButton(3) and *driveLatch) {
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
}