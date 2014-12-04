#Header
    /**
     * Group number: 28
     * Student 1:
     * Jente Hidskes, 4335732
     * Student 2:
     * Piet van Agtmaal, 4321278
     */

#Deadlines
|Week|Assignment|Deliverable|
|----|----------|-----------|
|4| Requirements & Set up| Report of 1 A4|
|5|ROS tutorials|none|
|6|Arduino|none|
|7|Line follower|none|
|8|Integration|Final report of 2-3 A4| 

#Passing criteria
* C: The robot can drive around based on *geometry_msgs/Twist* messages published on the topic *cmd_vel*.
* C: The robot will stop when an object of considerable size is placed in front of it.
* B: The robot is able to follow the line in test tracks 1-4.
* A: The timers in the Arduino sketch are set via the AVR timing registers and not with the Arduino libraries.
* A: The speed is adjusted based on the distance to the robot in front when several robots drive on the same test track.
