#include "Pixy.hpp"

PixyManager::PixyManager () {
    I2CPixy = new frc::I2C(frc::I2C::Port::kOnboard, I2CADDRESS);
    stick = new frc::Joystick{ 0 };
}

void PixyManager::pixy() {
    frc::SmartDashboard::PutBoolean("Pixy Connected", I2CPixy->AddressOnly());
}