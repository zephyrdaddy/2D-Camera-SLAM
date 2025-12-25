### Tutorial Scenario
1. 2D landmark auto generation / artificial generation 

2. 2D landmark class 
    Memory management 





## What I am interested in testing
1. How noisy sensor can be overcome with the optimization technique 
i.e) multi camera, number of features, 

2. Outliers  


가장 먼저 focus해서 보여줄건 뭐야. 

1. Spawn the world of 2D landmarks 

2. Generate some trajectory through the environment 

3. Estimate the robot pose + 2D landmark poses 

4. For the correspondence, just make a unique id for the landmarks.  

5. For the 2D projection search, first start with a brute force search on all the landmarks but maybe use more efficient type of data structure later

Pure visual odometry 

Multi dimension


Basic correctness
center point projects to 0 

scale invariance

left right symmetry
behind camera behavior

degenerate case y == 0 or very small



https://bazel.build/install/ubuntu

