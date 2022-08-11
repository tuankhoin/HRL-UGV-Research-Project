# ACCR Serial Communication Node

This node aims to build the communication between the onboard computing device and the robot platform. This communication node and protocol is developed dedicated to the ACCR UTGV platform and may be compatible with other platforms using the same protocol with a similar structure. The main function of this node is to subscribe to related ROS topics, pack them and send them to the low-level controller of the robot. In contrast, it is also in charge of receiving the feedback from the robot platform and publishing them to the corresponding ROS topics. The customised messages used in this package can be found in the [ACCR_msgs package](https://github.com/Murphy41/HRL-UGV-Research-Project/tree/main/src/system/ACCR_msgs "Amazing package!").

# Setup
Connect the UART pins on the low-level controller to the USB port of the onboard computing device via a USB-UART converter. The default parameters are defined as:
- baud rate: 115200bps
- information buffer size: 8 Byte
- stop bit: 1
- checksum: none

# Protocol
The communication protocol is defined as below.

## Handshake (3 steps)
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
    <td align="center" colspan="2"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#communication-cycle">Communication cycle</a></td>
    <td align="center"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#frequency-division">Frequency division</a></td>
    <td align="center"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#chassis-feedback-switch">Chassis feedback switch</a></td>
    <td align="center"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#bucket-feedback-switch">Bucket feedback switch</a></td>
    <td align="center"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#utility-feedback-switch">Utility feedback switch</a></td>
    <td align="center">N/D</td>
    <td align="center"><a href="https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#reset-feedback-switch">Reset switch</a></td>
  </tr>
</table>

For more details about these parameters, please see [below](https://github.com/Murphy41/HRL-UGV-Research-Project/edit/main/src/communication_layer/ACCR_Comm/README.md#initialization-parameters).



# Parameters
## Initialization parameters
### Communication cycle
Communication cycle (T) is the time cycle of each round of communication in ms.

### Frequency division
Frequency division, $k_f$ is the frequency division of the chassis info and other info. The chassis info communication frequency $f_{chas}$ is defined as $f_{chas} = \frac{1}{T}$. And the frequency of all other communications (currently including bucket and utilities) $f_{n-chas} = \frac{f_{chas}}{k_f}$.

### Chassis feedback switch
Chassis feedback switch (s<sub>chas</sub>) is a bit switch for chassis feedbacks. The first four bits from the MSB is not available (N.A.) in the current protocol, and the last four bits from the LSB corresponds to the chassis velocity feedback, chassis position feedback, chassis orientation feedback, and chassis torque feedback, as shown in the table below. At each bit, 0 means the correspoding feedback is not required and won't be sent. In contrast, 1 means the corresponding feedback will be sent.
<table>
  <tr>
    <th>Bit</th>
    <th>MSB</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>LSB</th>
  </tr>
  <tr>
    <td align="center">Chassis feedback</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">torque</td>
    <td align="center">orientation</td>
    <td align="center">position</td>
    <td align="center">velocity</td>
  </tr>
</table>

Lookup table for chassis feedback list:
<table>
  <tr>
    <th>Bit</th>
    <th>Hex</th>
    <th>Chassis feedback</th>
  </tr>
  <tr>
    <td align="center">0000</td>
    <td align="center">0x00</td>
    <td align="center">No feedback</td>
  </tr>
  <tr>
    <td align="center">0001</td>
    <td align="center">0x01</td>
    <td align="center">velocity</td>
  </tr>
  <tr>
    <td align="center">0010</td>
    <td align="center">0x02</td>
    <td align="center">position</td>
  </tr>
  <tr>
    <td align="center">0011</td>
    <td align="center">0x03</td>
    <td align="center">velocity+position</td>
  </tr>
  <tr>
    <td align="center">0100</td>
    <td align="center">0x04</td>
    <td align="center">orientation</td>
  </tr>
  <tr>
    <td align="center">0101</td>
    <td align="center">0x05</td>
    <td align="center">velocity+orientation</td>
  </tr>
  <tr>
    <td align="center">0110</td>
    <td align="center">0x06</td>
    <td align="center">position+orientation</td>
  </tr>
  <tr>
    <td align="center">0111</td>
    <td align="center">0x07</td>
    <td align="center">velocity+position+orientation</td>
  </tr>
  <tr>
    <td align="center">1000</td>
    <td align="center">0x08</td>
    <td align="center">torque</td>
  </tr>
  <tr>
    <td align="center">1001</td>
    <td align="center">0x09</td>
    <td align="center">velocity+torque</td>
  </tr>
  <tr>
    <td align="center">1010</td>
    <td align="center">0x0A</td>
    <td align="center">position+torque</td>
  </tr>
  <tr>
    <td align="center">1011</td>
    <td align="center">0x0B</td>
    <td align="center">velocity+position+torque</td>
  </tr>
  <tr>
    <td align="center">1100</td>
    <td align="center">0x0C</td>
    <td align="center">orientation+torque</td>
  </tr>
  <tr>
    <td align="center">1101</td>
    <td align="center">0x0D</td>
    <td align="center">velocity+orientation+torque</td>
  </tr>
  <tr>
    <td align="center">1110</td>
    <td align="center">0x0E</td>
    <td align="center">position+orientation+torque</td>
  </tr>
  <tr>
    <td align="center">1111</td>
    <td align="center">0x0F</td>
    <td align="center">velocity+position+orientation+torque</td>
  </tr>
</table>


### Bucket feedback switch
Bucket feedback switch (s<sub>bkt</sub>) is a bit switch for bucket feedbacks. The first six bits from the MSB is not available (N.A.) in the current protocol, and the last two bits from the LSB corresponds to the bucket mode feedback and bucket dynamics feedback as shown in the table below. At each bit, 0 means the correspoding feedback is not required and won't be sent. In contrast, 1 means the corresponding feedback will be sent.
<table>
  <tr>
    <th>Bit</th>
    <th>MSB</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>LSB</th>
  </tr>
  <tr>
    <td align="center">Bucket feedback</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">mode</td>
    <td align="center">dyanmics</td>
  </tr>
</table>

Lookup table for bucket feedback list:
<table>
  <tr>
    <th>Bit</th>
    <th>Hex</th>
    <th>Bucket feedback</th>
  </tr>
  <tr>
    <td align="center">00</td>
    <td align="center">0x00</td>
    <td align="center">No feedback</td>
  </tr>
  <tr>
    <td align="center">01</td>
    <td align="center">0x01</td>
    <td align="center">mode</td>
  </tr>
  <tr>
    <td align="center">10</td>
    <td align="center">0x02</td>
    <td align="center">dynamics</td>
  </tr>
  <tr>
    <td align="center">11</td>
    <td align="center">0x03</td>
    <td align="center">mode+dynamics</td>
  </tr>
</table>

### Utility feedback switch
Utility feedback switch (s<sub>utl</sub>) is a bit switch for utility feedbacks. The first seven bits from the MSB is not available (N.A.) in the current protocol, and only the LSB corresponds to the utility state feedback as shown in the table below. At each bit, 0 means the correspoding feedback is not required and won't be sent. In contrast, 1 means the corresponding feedback will be sent.
<table>
  <tr>
    <th>Bit</th>
    <th>MSB</th>
    <th>1</th>
    <th>2</th>
    <th>3</th>
    <th>4</th>
    <th>5</th>
    <th>6</th>
    <th>LSB</th>
  </tr>
  <tr>
    <td align="center">Bucket feedback</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">N.A.</td>
    <td align="center">switch</td>
  </tr>
</table>

Lookup table for bucket feedback list:
<table>
  <tr>
    <th>Bit</th>
    <th>Hex</th>
    <th>Bucket feedback</th>
  </tr>
  <tr>
    <td align="center">00</td>
    <td align="center">0x00</td>
    <td align="center">No feedback</td>
  </tr>
  <tr>
    <td align="center">01</td>
    <td align="center">0x01</td>
    <td align="center">mode</td>
  </tr>
  <tr>
    <td align="center">10</td>
    <td align="center">0x02</td>
    <td align="center">dynamics</td>
  </tr>
  <tr>
    <td align="center">11</td>
    <td align="center">0x03</td>
    <td align="center">mode+dynamics</td>
  </tr>
</table>





To use: Include the following line in your launch file:
  <node name="my_serial_node" pkg="my_serial_node" type="my_serial_node" />


Details to be added.

## Acknoledgement
This node relys on the Serial Communication Library developed by William Woodall, John Harrison and other contributors. For more details, please refer to their [GitHub page](https://github.com/wjwwood/serial)
