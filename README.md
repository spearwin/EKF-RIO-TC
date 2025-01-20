# EKF-Based Radar-Inertial Odometry with Online Temporal Calibration (EKF-RIO-TC)

This repository contains the implementation of **EKF-Based Radar-Inertial Odometry with Online Temporal Calibration (EKF-RIO-TC)**. The corresponding paper is currently **under review.** 

The code will be made publicly available after the review process is complete.

## Experimental Setup
The experimental setup includes the following components:
- **Radar:** Texas Instruments AWR1843BOOST.
- **IMU:** Xsens MTI-670-DK.
- **Ground Truth:** Provided using an OptiTrack motion capture system.
<img src="https://github.com/user-attachments/assets/ec7b5533-3f71-49e1-9a4c-3ba9ed422c2b" alt="Experimental_Setup" width="500">

## Datasets
- **Self-collected Dataset**  
  The self-collected dataset consists of a total of 7 sequences, including data from radar, IMU, and ground truth provided by an OptiTrack motion capture system.  
  - **Sequence 1–3:** Contain minimal rotational motion.  
  - **Sequence 4–7:** Contain significant rotational motion.  

  It is available at [Dataset]().

- **Open Datasets**  
  Two publicly available datasets were used for testing:
  - [ICINS2021 Dataset](https://christopherdoer.github.io/datasets/icins_2021_radar_inertial_odometry)  
  - [ColoRadar Dataset](https://arpg.github.io/coloradar/)
<img src="https://github.com/user-attachments/assets/c05b33c9-a520-40b0-a2bc-56669b038ad3" alt="Trajectory" width="800">

**Black**: Ground truth, **Blue**: EKF-RIO, **Red**: EKF-RIO-TC (proposed).

## References

## Contact
 

## Acknowledgement
- This work builds upon the implementation of [EKF-RIO](https://github.com/christopherdoer/rio).
- The trajectory evaluation in this project was conducted using the [evo](https://github.com/MichaelGrupp/evo) tool.

## License
- [GPLv3](https://www.gnu.org/licenses/) License.
