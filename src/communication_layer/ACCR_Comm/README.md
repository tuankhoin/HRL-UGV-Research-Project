# ROS Serial Communication Node

## Introduction
This node aims to build the communication bewtween the onboard computing device and the robot platform. This communication node and protocol is developed dedicated to the ACCR UTGV platform and may be compatible with other platform using the same protocol with similar structure. The main function of this node is to subscribe related ROS topics, pack them and send them to the low-level controller of the robot. In contrast, it also in charge of receive the feedback from the robot platform and publish them to the corresponding ROS topics. The customised messsages used in this package can be found in [ACCR_msgs package](https://github.com/Murphy41/HRL-UGV-Research-Project/tree/main/src/system/ACCR_msgs).

## Setup
Connect the UART pins on the low-level controller to the USB port of the on-board computing device via a USB-UART converter. The default parameters are defined as:
- baud rate: 115200bps
- information buffer size: 8 
- stop bit: 1
- checksum: none

## Protocol
The communication protocol is defined as below.

### Handshake (3 steps)
- Step 1 - Send the hand-shake message to the robot:

<table>
  <tr>
    <th>0</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>7</th>
    <th>8</th>
    <th>9</th>
  </tr>
  <tr>
    <td>0xAA</td>
    <td>0x00</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
  </tr>
</table>

- Step 2 - Receive the hand-shake feedback from the robot:

<table>
  <tr>
    <th>0</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>7</th>
    <th>8</th>
    <th>9</th>
  </tr>
  <tr>
    <td>0x55</td>
    <td>0x01</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
  </tr>
</table>

- Step 3 - Send initialization command to the robot:

<table>
  <tr>
    <th>0</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>7</th>
    <th>8</th>
    <th>9</th>
  </tr>
  <tr>
    <td>0xAA</td>
    <td>0x01</td>
    <td>Th</td>
    <td>Tl</td>
    <td>p1</td>
    <td>p2</td>
    <td>p3</td>
    <td>p4</td>
    <td>0</td>
    <td>reset</td>
  </tr>
  <tr>
    <td>Sending header</td>
    <td>Initializatin header</td>
    <td colspan="2">Communication frequency<sup>*1</sup></td>
    <td>Pre-scalar<sup>*2</sup></td>
    <td>Chassis feedback options<sup>*3</sup></td>
    <td>Bucket feedback options<sup>*4</sup></td>
    <td>Other options<sup>*5</sup></td>
    <td>N.A.</td>
    <td>Reset options<sup>*6</sup></td>
  </tr>
</table>

[^1]: The frequency differences between chassis info and other info in times.

To use: Include the following line in your launch file:
  <node name="my_serial_node" pkg="my_serial_node" type="my_serial_node" />


Details to be added.

## Acknoledgement
This node relys on the Serial Communication Library developed by William Woodall, John Harrison and other contributors. For more details, please refer to their [GitHub page](https://github.com/wjwwood/serial)
