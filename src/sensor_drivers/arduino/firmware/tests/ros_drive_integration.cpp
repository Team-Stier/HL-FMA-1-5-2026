// Executes the actual .ino + RosBridge + vehicle-specific core, with fake pins
// and serial transport. Message classes come from the generated real ros_lib.
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <cstring>
#include <Arduino.h>
uint32_t testMillis=0;
int testPwm[32]={};
uint8_t PINC=0, PCMSK1=0;
#include STIER_SKETCH

#define CHECK(expr) do { if (!(expr)) throw std::runtime_error( \
    std::to_string(__LINE__) + ": " #expr); } while (false)

void receive(uint16_t kph, uint8_t gear, bool brake=false, bool estop=false) {
  erp42_msgs::DriveCmd command;
  command.KPH=kph; command.Deg=-10; command.Gear=gear;
  command.brake=brake; command.EStop=estop;
  ros::Subscriber<erp42_msgs::DriveCmd>::callback(command);
}

void tick(float speed, bool resend, uint16_t kph=2, uint8_t gear=0,
          bool brake=false, bool estop=false) {
  testMillis+=20;
  if (resend) receive(kph,gear,brake,estop);
  updateRosCommand(testMillis);
  activeCommand=rosCommand;
  const bool freshEncoder=testMillis%100==0;
  if (freshEncoder) {
    frontEncoderMeasurement.speedMps=speed;
    frontEncoderMeasurement.deltaCount=speed>0 ? 3 : speed<0 ? -3 : 0;
    measuredAbsoluteKph=std::abs(speed)*3.6f;
    latestEncoderSampleTimeMs=100;
  }
  updateRosSpeedControl(testMillis,freshEncoder);
  updateSafety(testMillis);
  updateDrive(testMillis);
  publishOrPrintStatus(testMillis);
}

void hold() {
  for (int i=0;i<50;++i) {
    tick(0.,true,0,1,true);
    CHECK(latestFrontDrivePwm==0 && latestRearDrivePwm==0);
    CHECK(testPwm[BroonT870Controller::kFrontDrivePins.pwm]==0);
  }
  CHECK(safetyState.state==BroonT870::STATE_ARMED);
}

int main() {
 try {
  erp42_msgs::DriveCmd wire;
  wire.KPH=2; wire.Deg=-10; wire.brake=0; wire.Gear=2; wire.EStop=1;
  unsigned char packet[16]={};
  CHECK(wire.serialize(packet)==7);
  CHECK(std::strcmp(wire.getMD5(),"518982e31d00755722fd5fb8c3000c77")==0);
  erp42_msgs::DriveCmd decoded;
  CHECK(decoded.deserialize(packet)==7);
  CHECK(decoded.KPH==2 && decoded.Deg==-10 && decoded.Gear==2 && decoded.EStop==1);
  setup();
  selectedMode=MODE_ROS; rcStopActive=false;
  hold(); // Control's normal brake command must not cause rearming deadlock.
  receive(65535,2);
  updateRosCommand(testMillis);
  CHECK(rosCommand.targetSpeedKph==-5.f);
  receive(65535,0);
  updateRosCommand(testMillis);
  CHECK(rosCommand.targetSpeedKph==15.f);
  for (int i=0;i<30;++i) tick(0.,true);
  CHECK(latestFrontDrivePwm>0 && latestRearDrivePwm>0);
  for (int i=0;i<10;++i) tick(.3,true);
  // Reverse requests while still rolling forward must first remove PWM.
  for (int i=0;i<15;++i) {
    tick(.3,true,2,2);
    CHECK(latestFrontDrivePwm==0 && latestRearDrivePwm==0);
    CHECK(rosDirection.accepted==1);
  }
  for (int i=0;i<10;++i) {
    tick(0.,true,2,2);
    if (rosDirection.stationarySamples<3) {
      CHECK(latestFrontDrivePwm==0);
      CHECK(rosDirection.accepted==1);
    }
  }
  for (int i=0;i<30;++i) tick(0.,true,2,2);
  CHECK(rosDirection.accepted==-1);
  CHECK(latestFrontDrivePwm<0 && latestRearDrivePwm<0);
  for (int i=0;i<10;++i) tick(-.3,true,2,2);
  auto* feedback=dynamic_cast<erp42_msgs::SerialFeedBack*>(ros::lastPublished);
  CHECK(feedback && feedback->MorA==1 && feedback->Gear==2);
  CHECK(feedback->speed<0 && feedback->encoder<0 && feedback->EStop==0);
  // Old samples cannot be counted as three new zero-speed observations.
  StierRosDrive::DirectionState policy{1,0};
  for (int i=0;i<100;++i) StierRosDrive::observe(policy,false,0,0,0,0);
  CHECK(!StierRosDrive::select(policy,-1));
  hold();
  CHECK(feedback->EStop==0 && feedback->Gear==1 && feedback->brake!=0);
  for (int i=0;i<30;++i) tick(0.,true);
  CHECK(latestFrontDrivePwm>0); // reverse -> hold -> forward
  for (int i=0;i<10;++i) {
    tick(.3,true,2,0,false,true);
    CHECK(latestFrontDrivePwm==0 && !outputsAllowed);
  }
  CHECK(feedback->EStop==1);
  hold(); // explicit EStop=0 releases; normal brake is not emergency
  CHECK(feedback->EStop==0);
  for (int i=0;i<10;++i) {
    tick(0.,true,0,0); // zero magnitude is a stop even with forward gear and brake=0
    CHECK(latestFrontDrivePwm==0 && latestRearDrivePwm==0);
  }
  for (int i=0;i<30;++i) tick(0.,true);
  CHECK(latestFrontDrivePwm>0);
  for (int i=0;i<65;++i) tick(.3,false);
  CHECK(!activeCommand.valid && latestFrontDrivePwm==0);
  hold();
  for (int i=0;i<10;++i) tick(0.,true,2,42);
  CHECK(!activeCommand.valid && latestFrontDrivePwm==0);
  // Parking emits integer 1 km/h, not the 2 km/h used above. Repeat normal
  // holds and both directions to expose stale arming/gear/PI state between legs.
  // Pin output is verified; this cannot establish real motor breakaway torque.
  for (int cycle=0; cycle<20; ++cycle) {
    hold();
    const uint8_t gear=cycle%2==0 ? 2 : 0;
    const int sign=gear==2 ? -1 : 1;
    for (int i=0;i<30;++i) tick(0.,true,1,gear);
    CHECK(sign*latestFrontDrivePwm>0 && sign*latestRearDrivePwm>0);
    for (int i=0;i<20;++i) tick(sign*.2f,true,1,gear);
    CHECK(safetyState.state==BroonT870::STATE_ARMED && outputsAllowed);
    CHECK(feedback->Gear==gear && feedback->MorA==1 && feedback->EStop==0);
    CHECK(sign*feedback->speed>0 && sign*feedback->encoder>0);
  }
  hold();
  rcStopActive=true;
  tick(0.,true,2,0);
  CHECK(latestFrontDrivePwm==0 && !outputsAllowed);
  // The unchanged core's RC neutral/brake and direction interlock semantics.
  DrivePairState rcDrive{};
  auto forward=BroonT870::updateDrivePair(rcDrive,100,230,100,10,20,300,0,false);
  CHECK(forward.frontPwm>0);
  auto reverse=BroonT870::updateDrivePair(rcDrive,-100,230,100,10,20,300,20,false);
  CHECK(reverse.frontPwm==0);
  reverse=BroonT870::updateDrivePair(rcDrive,-100,230,100,10,20,300,320,false);
  CHECK(reverse.frontPwm<0);
  std::cout << "PASS: wire MD5/serialization, bridge, brake/rearm, signed F/R, standstill shift, limits, feedback, EStop, timeout, RC stop/interlock, 20 alternating parking legs at 1 km/h\n";
  return 0;
 } catch(const std::exception& e) {
  std::cerr << e.what() << '\n'; return 1;
 }
}
