# ROS Serial Communication Node

## Introduction
This node aims to build the communication between the onboard computing device and the robot platform. This communication node and protocol is developed dedicated to the ACCR UTGV platform and may be compatible with other platforms using the same protocol with a similar structure. The main function of this node is to subscribe to related ROS topics, pack them and send them to the low-level controller of the robot. In contrast, it is also in charge of receiving the feedback from the robot platform and publishing them to the corresponding ROS topics. The customised messages used in this package can be found in the [ACCR_msgs package](https://github.com/Murphy41/HRL-UGV-Research-Project/tree/main/src/system/ACCR_msgs).

## Setup
Connect the UART pins on the low-level controller to the USB port of the onboard computing device via a USB-UART converter. The default parameters are defined as:
- baud rate: 115200bps
- information buffer size: 8 Byte
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
    <td align="center">0xAA</td>
    <td align="center">0x00</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
  </tr>
  <tr>
    <td align="center">Sending header</td>
    <td align="center">Hand-shake header</td>
    <td colspan="8" align="center">N/D</td>
  </tr>
</table>
</p>

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
    <td align="center">0x55</td>
    <td align="center">0x01</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
    <td align="center">0</td>
  </tr>
  <tr>
    <td align="center">Receiving header</td>
    <td align="center">Hand-shake header</td>
    <td align="center" colspan="8">N/D</td>
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
    <td align="center">0xAA</td>
    <td align="center">0x01</td>
    <td align="center" colspan="2">T</td>
    <td align="center">$k_f$</td>
    <td align="center">s<sub>chas</sub></td>
    <td align="center">s<sub>bkt</sub></td>
    <td align="center">s<sub>utl</sub></td>
    <td align="center">0</td>
    <td align="center">s<sub>rst</sub></td>
  </tr>
  <tr>
    <td align="center">Sending header</td>
    <td align="center">Initialization header</td>
    <td align="center" colspan="2">Communication cycle<sup></sup></td>
    <td align="center">Frequency division<sup></sup></td>
    <td align="center">Chassis feedback switch<sup></sup></td>
    <td align="center">Bucket feedback switch<sup></sup></td>
    <td align="center">Utility feedback switch<sup></sup></td>
    <td align="center">N/D</td>
    <td align="center">Reset switch<sup></sup></td>
  </tr>
</table>

- Communication cycle, T: the time cycle of each round of communication in ms.
- Frequency division, $k_f$: the frequency division of the chassis info and other info. The chassis info communication frequency $f_{chas}$ is defined as $f_{chas} = \frac{1}{T}$. And the frequency of all other communications (currently including bucket and utilities) $f_{n-chas} = \frac{f_{chas}}{k_f}$.
- Chassis feedback switch: a bit switch for chassis feedback. The bit switch and corresponding 



[^1]: The frequency differences between chassis info and other info in times.

To use: Include the following line in your launch file:
  <node name="my_serial_node" pkg="my_serial_node" type="my_serial_node" />


Details to be added.

## Acknoledgement
This node relys on the Serial Communication Library developed by William Woodall, John Harrison and other contributors. For more details, please refer to their [GitHub page](https://github.com/wjwwood/serial)
