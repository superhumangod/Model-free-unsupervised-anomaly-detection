# Model-free-unsupervised-anomaly-detection

This is a source code for "Model-Free Unsupervised Anomaly Detection of a General Robotic System Using a Stacked LSTM and Its Application to a Fixed-Wing Unmanned Aerial Vehicle", International Conference on Intelligent Robots and Systems (IROS 2022).

1) Data
We used ALFA (A Dataset for UAV Fault and Anomaly Detection) dataset from\
https://kilthub.cmu.edu/articles/dataset/ALFA_A_Dataset_for_UAV_Fault_and_Anomaly_Detection/12707963 \
Detailed explanation of the data is in\
http://theairlab.org/alfa-dataset/ \
If you are using the data in this repository, please cite\
A. Keipour, M. Mousaei, and S. Scherer, “ALFA: A dataset for UAV fault and anomaly detection,” The International Journal of Robotics Research, vol. 0. no.  0,  pp.  1–6,  October  2020.  [Online]. Available:https://doi.org/10.1177/0278364920966642

2) For_processing_ALFA data.m\
This is a matlab code.\
We utilized the original code in ALFA dataset to make this code.\
We used this code to process ALFA dataset for our purpose.

3) Stacked LSTM folder\
3-1) data_LSTM folder\
Data for learning the code.\
3-2) 20220114_the stacked LSTM.ipynb\
This is a jupyter lab file written in Python 3.7 and tensorflow 2.3.1\
In this code, chapter 1~3 is our model.\
Chapter 4 is the code for learning the reference [21] in our paper.\
Chapter 5 is for comparing response times.\
3-3) 20220113_L=4_model.47-0.1205.h5\
Best stacked LSTM model in our research.

4) ICRA paper folder\
4-1) data folder\
Data for learning the code\
4-2) ICRAsim.m\
This is a matlab file.\
This code is for learning the reference [11] in our paer.
