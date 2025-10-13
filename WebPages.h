#ifndef WEBPAGES_H
#define WEBPAGES_H

#include <Arduino.h>

// Function declarations for web page handlers
String generateMainPage(String urlMain, String ip);
String generateRobotControlPage(int lastTheta1, int lastTheta2, int lastTheta3, 
                               int lastGripper, int lastSpeed, int lastAcceleration,
                               double endEffectorX, double endEffectorY, double endEffectorZ);
String generateJointControlPage(int lastTheta1_joint, int lastTheta2_joint, int lastTheta3_joint,
                               int lastGripper, int lastSpeed, int lastAcceleration,
                               double endEffectorX, double endEffectorY, double endEffectorZ);
String generateGcodePage();
String generateSetupPage();
String generateConfigPage(float theta1AngleToSteps, float theta2AngleToSteps, float phiAngleToSteps,
                         double L1, double L2, double baseHeight);
String generateOtaPage();
String generateApiDocPage();

// Function to read string from EEPROM (declared here for use in config page)
String readStringFromEEPROM(int addr);

#endif