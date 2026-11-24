#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}


///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

/*void test_1() {
  chassis.pid_drive_set(10_in, DRIVE_SPEED); //(distance, speed) goes forward
  chassis.pid_wait(); //waits for action
  lift.move_absolute(100, 127); //(position, speed) goes up
  chassis.pid_drive_set(-10_in, DRIVE_SPEED); //goes back to starting position
  chassis.pid_wait();
  lift.move_absolute(0, 127); //goes down to starting position 

}*/

// PT&: might delete later 
// void test_auto() {
//   chassis.pid_drive_set(10_in, DRIVE_SPEED); //(distance, speed) goes forward
//   chassis.pid_wait(); //waits for action
  
//   lift.move_absolute(100, 127); //(position, speed) goes up
//   claw.set(true); // PT&: questionable
//   chassis.pid_drive_set(-10_in, DRIVE_SPEED); //goes back to starting position
//   chassis.pid_wait();
//   lift.move_absolute(0, 127); //goes down to starting position 
//   claw.set(false); // PT&: questionable
//   chassis.pid_wait();

//   chassis.pid_drive_set(15_in, DRIVE_SPEED); //(distance, speed) goes forward
//   chassis.pid_wait(); //waits for action

//    lift.move_absolute(100, 127); //(position, speed) goes up
//   claw.set(true); // PT&: questionable
//   chassis.pid_drive_set(-10_in, DRIVE_SPEED); //goes back to starting position
//   chassis.pid_wait();
//   lift.move_absolute(0, 127); //goes down to starting position 
//   claw.set(false); // PT&: questionable
//   chassis.pid_wait();
//   chassis.pid_drive_set(-15_in, DRIVE_SPEED); //(distance, speed) goes backward

 



// }

// === PT&: 11/20 Route --> Turn right grab your stuff, place === //
 
// PT&: function that defines what our lift does in one motion
void lift_control() {
  lift.move(90);  // out of 127    
  pros::delay(500);
  lift.move(5); // big Jess said don't leave it at purely 0
}

void test_autonomous() {
  // initialize everything
  // chassis.pid_targets_reset();
  // chassis.drive_sensor_reset();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  // PT&&: never mind, it's redundant, the autonomous() on main already does so

  // === NOTES ON AUTON === //
  /**
   * Jess is goated
   * When use controller, don't click Timed Run. Click DOWN + B
   * So when you press Program ==> Drive, you controller should only vibrate once
   * if your controller vibrates 3 times, EZ-Template is telling you that your 
   * PID sensor isn't working, or other issue
   * but if it vibrates 1 time, you're fine. 
   * Always, always check your PID/IMU sensor, that it's plugged and glowing
   */

  // === Route Begins Proper Starting Below === //

  // == START from here == //


  // PT&: Route 1: start by just turn and grab from the little pod idk
  // first of all: the bot starts with the claw "open" upon firing up the code

  // start by driving backwards while trying to 
  
  // start up by 5 inches, to clear the side pins first
  // note: it's just at a constnat 80 speed, just testing a slow start
  // chassis.pid_drive_set(5_in, DRIVE_SPEED); // PT&&: here's just the most basic drive
  // chassis.pid_wait(); // wait for action

  // // turn right 90 degrees, if negative (-90) then turn left 90 degrees
  // chassis.pid_turn_set(90_deg,TURN_SPEED); //0 heading is straight turns right at a certain speed
  // chassis.pid_wait();

  // //Forward 57.5 inches align it horizontally with the dispenser
  // chassis.pid_drive_set(57.5_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // // turn another 90 degrees, have bot face the pin
  // chassis.pid_turn_set(90_deg,TURN_SPEED);
  // chassis.pid_wait();
  
  // // drive back towards the pin a tiny bit, to allign claw with the pin
  // chassis.pid_drive_set(2_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // // change the state of the claw, clamp that onto the pin
  // claw.set(true);
  // pros::delay(250); // PT&&: for the whole machine to pause for a moment, not just chassis

  // // move backwards a bit, give leeway for the future turn
  // chassis.pid_drive_set(-2_in, DRIVE_SPEED);
  // chassis.pid_wait();


  // // now the pin is in the cluthces of the claw, turn the bot around fully, to start placement
  // chassis.pid_turn_set(180_deg,TURN_SPEED);
  // chassis.pid_wait();

  // // now lift, just test if the claw holds onto the claw as it goes up.
  // lift_control();
  // chassis.pid_wait();


  // // == Peter Test 2: Test 2 == //
  // chassis.pid_drive_set(70_in, DRIVE_SPEED); //(distance, speed) goes forward
  // chassis.pid_wait(); //waits for action

  // // close the claw to grab the pin
  // claw.set(true);
  // //chassis.pid_wait_quick_chain();

  // // lift that john up
  // auton_cascade();

  // // drive backwards 25 inches to align with the red goal
  // chassis.pid_drive_set(-40_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // // turn left by 90 degrees to align self with the red goal
  // chassis.pid_turn_set(-90_deg,TURN_SPEED);
  // chassis.pid_wait();

  // // hold the pin up as you approach the red box
  // lift.move(90);

  // // move up to the red box
  // chassis.pid_drive_set(20_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // // let the pin in
  // pros::delay(500); 
  // lift.move(5);

  // // open the claw
  // // close the claw to grab the pin
  // claw.set(false);
  // //chassis.pid_wait_quick_chain();
  // pros::delay(500);


  // // back up to create some space
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // chassis.pid_wait();


  // Peter Test 3 -- change the toggle to red


  // === CSUN Sunrise Competition Auton === //
  // hold the claw
  // true sets to the opposite of the starting position
  // starting position is always "open" so setting claw to true always closes it
  claw.set(true); // PT&&:
  pros::delay(1000);


  // chassis.pid_drive_set(-1_in, DRIVE_SPEED);
  // chassis.pid_wait();

  
  //pros::delay(500);
  // chassis.pid_wait_until(-1_in);
 // chassis.pid_wait();

  lift.move(100);
  // chassis.pid_wait_quick_chain();
  pros::delay(1500);
  lift.move(-80);

  // you need your delays
  pros::delay(2000);
  lift.move(100);
  pros::delay(1500);
  // chassis.pid_wait_quick_chain();
  lift.move(-80);
  pros::delay(2000);

  
  // officially the end of trying the toggle.
  chassis.pid_wait();

  // Peter Test 4 - move to center of the map
  chassis.pid_drive_set(19_in, DRIVE_SPEED);
  chassis.pid_wait();

  // just adjust the lift to go up
  lift.move(100);
  pros::delay(1500);

  // turn self right by 90 degrees to align self with the red goal
  chassis.pid_turn_set(90_deg,TURN_SPEED);
  chassis.pid_wait();

  // go up to your team slot 
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);


  // throw that thing down
  lift.move(-80);
  pros::delay(1500);

  // let go of your claw
  claw.set(false);
  pros::delay(1000);

   // go back a bit
  chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  chassis.pid_wait();

  //turn back to align self with the center
  //chassis.pid_turn_set(-90_deg,TURN_SPEED); // might be absolte
  chassis.pid_turn_set(0_deg,TURN_SPEED);
  chassis.pid_wait();

// drive 10 inches to approach the red/blue
chassis.pid_drive_set(18_in, DRIVE_SPEED);
chassis.pid_wait();

// // close it around the red/blue, never knock that thang over, it has red on clear side
// claw.set(true);
// pros::delay(1000);


// === end of CSUN Sunrise Competition Auton === //









  



}
