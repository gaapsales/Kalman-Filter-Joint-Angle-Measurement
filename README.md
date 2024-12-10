# Kalman-Filter
Implemention of Kalman Filter to measure knee angles using data from BNO085 9-DoF IMU

This filter is one of the most used filters when it comes to calibrating an IMU device. Unlike conventional filters like LPF, HPF, and BPF, Kalman Filter is an algorithm used in predicting (or estimating) the next measurement using past measurement. Essentially, it serves as an estimator for predicting any state or component within a signal. The outcome of this estimation resembles the removal of noise from the signal [1]. The Kalman Filter undergoes a prediction and correction step where the filter predicts how noise may affect the future state of the system, and incorporates measurements from sensors to correct its state estimation:

![image](https://github.com/user-attachments/assets/aec31b6b-d904-49c8-b590-d52cd9c75a38)

Where X is the estimated state, A is the state transition matrix, P is the process matrix, Q is the process noise covariance matrix, H is the measurement matrix, R is the measurement noise covariance matrix, K is the Kalman gain, and Z is the Euler angles in quaternion. Small p rep resents the matrix that has been updated with a new prediction. Small t represents the time index. The equation provided represents the Kalman Filter model. However, it requires an ini tialization before it can fulfill its intended function. To apply the Kalman Filter effectively, certain initializations to the conditions are required. These adjustments include:

![image](https://github.com/user-attachments/assets/d175f9a2-b8d7-4f84-8a14-5df8c5c5ae3c)

Since this filter is an algorithm being used for estimating the state of a dynamic system from different noisy measurements, it helps in mitigating the effects of drift in some cases. It helps maintain an estimate of the true state of a system over time that includes parameters such as the position and velocity for a state estimation. However, the implementation of this said filter has one of its drawbacks of its requirement to fine tuning or optimizing the filter parameters internally in order to produce an accurate estimation [1]. Certain parameters, most specifically the R and Q matrices, are changed by some values and then analyzed in order to find what is the optimal value to get the best result. In this paper, the values of R and Q are 10 and 0.1, respectively. As indicated by Maarif et al., the bigger the ratio between R and Q values, the bigger the noise damping effect is. A ratio difference of 100 exhibits a minimal original data loss, but also exhibiting a smoother filter result.

# Results

One of the sensor fusion algorithms that was implemented is the Kalman filter. The Kalman filter, as mentioned previously, is a state estimator that can extract information from noisy data. It reduces the uncertainty by combining the gyroscope, accelerometer, and magnetometer data to generate a more reliable output. This filter was implemented to both IMU devices located at the thigh, and shank to determine the orientation of these devices during the squat movement. The f iltered data of the two IMU, specifically the pitch angles, was used in determining the knee joint angle

![image](https://github.com/user-attachments/assets/9a1332c1-7fe1-4a1a-a090-5be3dc74e1b7)

The Figure 7.8 illustrates the comparison between knee angles derived from both the Vi con and the Kalman filter algorithms. The findings demonstrate a slightly similar performance between the two, indicating the accuracy of knee flexion angle estimation using the Kalman filter. An evident characteristic of the Kalman filter is its capability to smooth out angles, particularly at the knee flexion peak, in contrast to the other two filters. Despite the Kalman filter providing fairly accurate estimations, it displayed noticeable overshooting at peak angles, primarily due to the relative motion of the IMU compared to the participant’s movement. A ratio difference of 100, as shown in Figure 7.8, is applied between the process noise and measurement noise matrices. If the noise is underestimated, the filter may excessively integrate noise, resulting in noisy estimations. This is shown in Fig 7.9, wherein the ratio difference between the process noise and measurement noise matrices is 1. Conversely, overestimating the noise may cause the filter to be overly cautious, leading to slower response times or reduced accuracy; this is shown in Fig 7.10.

![image](https://github.com/user-attachments/assets/4ec084b4-882c-4caf-8d74-869b60d09e9e)

![image](https://github.com/user-attachments/assets/c03f5c78-3f08-45c1-ba81-6b536954c326)

