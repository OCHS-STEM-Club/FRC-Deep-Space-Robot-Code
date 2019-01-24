#include "Pixy.hpp"

PixyManager::PixyManager () {
    I2CPixy = new frc::I2C(frc::I2C::Port::kOnboard, I2C_ADDRESS);
    stick = new frc::Joystick{ 0 };

    thing = new double;
    *thing = 0;
}

void PixyManager::pixy() {
    frc::SmartDashboard::PutBoolean("Pixy Connected", I2CPixy->AddressOnly());
    frc::SmartDashboard::PutNumber("Pixy Test Read", I2CPixy->Read(I2C_ADDRESS, 31, buff));
    
}