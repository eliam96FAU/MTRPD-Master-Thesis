# MTRPD-Master-Thesis of Eliam Kourie @FAU

Master thesis title: 
Optimization of the Multi-trip Travelling Repairman Problem with the Assistance of Drones.

Respository description: 
Three .py files to solve the MTRPD and one instances folder are located in this repository:
    1. MTRPD.py: The mathematical formulation is built, and Gurobi is used to solve the problem without any modifications.
    2. MTPRD_LBBD.py: The MTRPD is solved via the decomposition algorithm published by Bruni et al. (https://doi.org/10.1016/j.cor.2022.105845).
    3. MTRPD_LBBD_CL.py: The algorithm explained in the thesis is implemented to solve the MTRPD.
    4. Instances: 4 typse of instances, 40 in total, having one depot and ten customers. Those instances are used in Bruni et al. (https://doi.org/10.1016/j.cor.2022.105845)

Installation instructions: 
The packages in each .py file need to be downloaded to run the algorithms.
The input is an instance with nodes distributed on a [0, 100]x[0, 100] landscape. 
The instance should be in the same form as the examples in the 'Instances' folder, i.e. the depot should be located in the same line of the file, and the customer locations start from the same line.
If new instances should be tested: The path of the new instances 'instancePath' in the 'main()' function should be edited accordingly.

Usage examples:
In the 'main()' function of each .py file you can find the examples to run the algorithm. The general parameters and the path of the instance 'instancePath' need to be adjusted accordingly.

Algorithm implementation details, performance, limitations, and License information: 
All details are described in the thesis file.

Contact information for questions, remarks, and feedback: 
Email address: eliamkourie@hotmail.com.





