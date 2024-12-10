# Kalman-Filter
Implemention of Kalman Filter to measure knee angles using data from BNO085 9-DoF IMU

This filter is one of the most used filters when it comes to calibrating an IMU device. Unlike conventional filters like LPF, HPF, and BPF, Kalman Filter is an algorithm used in predicting (or estimating) the next measurement using past measurement. Essentially, it serves as an estimator for predicting any state or component within a signal. The outcome of this estimation resembles the removal of noise from the signal [49]. The Kalman Filter undergoes a prediction and correction step where the filter predicts how noise may affect the future state of the system, and incorporates measurements from sensors to correct its state estimation:

![image](https://github.com/user-attachments/assets/aec31b6b-d904-49c8-b590-d52cd9c75a38)
