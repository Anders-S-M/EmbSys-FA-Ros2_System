# Emb-Sys-FA-Ros2_System
# Embedded Systems Final Assignment Ros2 System ++

This repository contains the Ros2 system of our embedded systems final assignment.

In the src folder is the packages for the ros2 system. To run this add this to you ros2 workspace directory, then, while having sourced ros2 run colcon build then the nodes should be ready to run. It is built for ros2 foxy and validated in Ubuntu 20.04.

In the HLS SRC folder is the source code for the CNN network IP. To run this add the code to Vitis HLS. Use inference as your top function. Validate the functionality by using the test bench called CNN_tb.

The inferenceIP is our synthesized component made from the HLS SRC code.

