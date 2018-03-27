# A*
## The A* (A-Star) algorithm in MATLAB. 

The function __a_star__ accepts a logical 2d matrix that represents a map. TRUE specifies a visitable map cell, and FALSE indicates that a map cell cannot be visited. The algorithm finds the shortest path through the map. A _cost_ may be specified for each map cell, which penalizes visits to that cell. The cost can be used to represent that a given cell is farther away from others, or takes long to traverse, etc.

See __test_a_star__ for examples.

Running __test_a_star__ will execute a number of tests and generate figures.

![](./images/001.png)

![](./images/002.png)

![](./images/003.png)

