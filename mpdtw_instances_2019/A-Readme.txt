

Instances for the multi-pickup and delivery problem with time windows used in the paper: 

I. Aziez, J.-F. Côté, L.-C.Coelho, A branch abd cut algorithm for the multi-pickup and delivery problem with time windows, submitted, 2019.


** Instance type**

Each instance type is defined by three main elements: the TW type (Large, Normal, Without), the maximum length of the requests (4 or 8), 
and the number of nodes (instance size: 25, 35, 50, 100 customers).

An example about how to read the instance's type: the instances of type L_8_25 represent Large TW (L), Long requests (8), 
and 25 nodes.

**Instance data**

The data in each instance file is organized as follows:

    First line: Number of vehicles , capacity of each vehicle.

    Remaining lines: node ID, coordinate X, coordinate Y, demand, time windows start, time windows end, service time, node type (0 = pickup node , 1 = delivery node), request ID.


**Note**: note that the second line represents the depot (with a request ID = -1), and the remaining lines represent pickup and delivery nodes.
