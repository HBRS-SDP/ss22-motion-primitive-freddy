**Meeting notes:-**

Date :- 14/04/2022

- The main mode of communication between the robot and the pc is lancable.
- kelorobot.c is a file that returns torque.
- lapack and CBLAS is a library used to solve complex trigonometric problems.
- GNU Scientific Library uses lapack to solve complex trigonometric problems.
- Program explanation of Repository: https://github.com/HBRS-SDP/ws21-kelo-500-motion-control/tree/main/KELO_SDP and File: KELO_SDP -> EthercatCommunication.c :-
   * Line 81 of code represents slave ID.
   * Line 93 of code represents laptop's port ID and it is different for Macbook.
   * Lines 126-145 of the code contain a function that is used for communication. It checks which slave is working which is not and also checks whether the robot has reached the safe operating state.
   * Line 147 of code is for initialising message.
   * Lines 150-153 of code have variables limit 1 and limit 2 which represent the maximum and minimum limits for the wheels.
   * Lines 154-155 of code have variables setpoint1 and setpoint2 which represent the curremt values.
   * Line 157-161 of code is for reading the message and pasting that message into the EC data.
   * Lines 179-180 of code is for sending and receiving data.
   * From line 204 of code is the implementation of the force control. 
   * Line 225 of code has a variable pivot_angles which stores pivot angles.
   * Line 226 of the code has a variable wheel_torque that stores the torque of all 8 wheels.
   * Line 240 of code reads angles of the wheels.
   * To deactivate the wheels , make the diagonal of a matrix zero.
   * Values of a matrix is called weights.
   * Line 227 of the code passes all the initial values and sets the mode as torque mode.
   * Setpoint 1 is always negative and setpoint 2 is always positive.
   * From line 315 line of the code, the message has been constructed. In this message, data has been sent, and feedback is received. In feedback, pivot angles have been used, and these angles have been used to calculate wheel torques.
  