# EKF-Based Radar-Inertial Odometry with Online Temporal Calibration (EKF-RIO-TC)

This repository contains the implementation of **EKF-Based Radar-Inertial Odometry with Online Temporal Calibration (EKF-RIO-TC)**.  
The corresponding paper has been accepted for publication in **IEEE Robotics and Automation Letters (RA-L).**  

## Prerequisites
Make sure the following are installed:
- ROS (e.g., Noetic or Melodic)
- `catkin_tools`
- [catkin_simple](https://github.com/catkin/catkin_simple):
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/catkin/catkin_simple.git
### Build and Run

```bash
# Clone the repository and its submodules
cd ~/catkin_ws/src
git clone https://github.com/spearwin/EKF-RIO-TC.git
cd EKF-RIO-TC
git submodule update --init --recursive

# Build the workspace
cd ~/catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source ~/catkin_ws/devel/setup.bash
```

To launch the system, choose one of the following based on your use case:

- **If you are using rosbag playback:**
  ```bash
  roslaunch ekf_rio_tc <dataset_name>_rosbag.launch
  ```

- **If you are using live ROS topics:**
  ```bash
  roslaunch ekf_rio_tc <dataset_name>.launch
  ```
  or play the bag file manually:
  ```bash
  rosbag play <dataset_path>/<sequence>.bag
  ```

## Experimental Setup
The experimental setup includes the following components:
- **Radar:** Texas Instruments AWR1843BOOST.
- **IMU:** Xsens MTI-670-DK.
- **Ground Truth:** Provided using an OptiTrack motion capture system.
<img src="https://github.com/user-attachments/assets/ec7b5533-3f71-49e1-9a4c-3ba9ed422c2b" alt="Experimental_Setup" width="500">

### Sensor Setup
For the Texas Instruments AWR1843BOOST radar sensor, configuration is performed via the [mmWave Demo Visualizer](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/). In our self-collected dataset, the key radar parameters are as follows:
- **Frame Rate:** 10 fps
- **Doppler Velocity Resolution:** 0.06 m/s
- **Range Resolution:** 0.04 m
- **CFAR/Doppler Range Threshold:** 10 dB (this parameter may need adjustment based on environmental conditions)

## Datasets
- **Self-collected Dataset**  
  The self-collected dataset consists of a total of 7 sequences, including data from radar, IMU, and ground truth provided by an OptiTrack motion capture system.  
  - **Sequence 1–3:** Involve smoother motion with smaller gyroscope changes over the time offset interval.  
  - **Sequence 4–7:** Exhibit greater variation, leading to a larger radar ego-velocity discrepancy and clearer impact of the time offset.

  The dataset is included in the repository under the following path:
  ```
  EKF-RIO-TC/ekf_rio_tc/datasets
  ```
  
- **Open Datasets**  
  Two publicly available datasets were used for testing:
  - [ICINS2021 Dataset](https://christopherdoer.github.io/datasets/icins_2021_radar_inertial_odometry)  
  - [ColoRadar Dataset](https://arpg.github.io/coloradar/)
<img src="https://github.com/user-attachments/assets/c05b33c9-a520-40b0-a2bc-56669b038ad3" alt="Trajectory" width="800">

**Black**: Ground truth, **Blue**: EKF-RIO, **Red**: EKF-RIO-TC (proposed).

## References
- C. Kim, G. Bae, W. Shin, S. Wang and H. Oh, "EKF-Based Radar-Inertial Odometry With Online Temporal Calibration," in IEEE Robotics and Automation Letters, vol. 10, no. 7, pp. 7230-7237, July 2025, [DOI Link](https://doi.org/10.1109/LRA.2025.3575320).

## Contact
- **Changseung Kim**  
Email: pon02124@unist.ac.kr

## Acknowledgement
- This work builds upon the implementation of [EKF-RIO](https://github.com/christopherdoer/rio).
- The trajectory evaluation in this project was conducted using the [evo](https://github.com/MichaelGrupp/evo) tool.

## License
- [GPLv3](https://www.gnu.org/licenses/) License.
