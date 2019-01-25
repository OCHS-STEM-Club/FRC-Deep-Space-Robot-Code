#include "Vision.hpp"

PixyManager::PixyManager () {
    I2CPixy = new frc::I2C(frc::I2C::Port::kOnboard, I2C_ADDRESS);
    stick = new frc::Joystick{ 0 };
    // translate = new int;
    
}

void PixyManager::pixy() {
    byte buff[31];
  I2CPixy->Read(0x64, 31, buff);

  if (!(buff[1] == buff[2])) {
    if (buff[0] == 0) {
      for(int i = 0; i < 30; i++) {
        buff[i] = buff[i + 1];
      }
    }
  }

  for(int i = 0; i < 15; i++) {
    translate[i] = buff[(2 * i) + 1] * 256 + buff[2 * i];
  }

  frc::SmartDashboard::PutNumber("No Checksum Sync Byte 1", translate[0]);
  frc::SmartDashboard::PutNumber("No Checksum Sync Byte 2", translate[1]);
  frc::SmartDashboard::PutNumber("Packet Type Byte", translate[2]);
  frc::SmartDashboard::PutNumber("Data Length Byte", translate[3]);
  frc::SmartDashboard::PutNumber("Pixy X1", translate[4]);
  frc::SmartDashboard::PutNumber("Pixy Y1", translate[5]);
  frc::SmartDashboard::PutNumber("Pixy W1", translate[6]);
  frc::SmartDashboard::PutNumber("Pixy H1", translate[7]); 
  frc::SmartDashboard::PutNumber("Pixy X2", translate[12]);
  frc::SmartDashboard::PutNumber("Pixy Y2", translate[12]);
  frc::SmartDashboard::PutNumber("Pixy W2", translate[13]);
  frc::SmartDashboard::PutNumber("Pixy H2", translate[14]);
}