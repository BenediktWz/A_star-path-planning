# A_star-path-planning

## Description:
In this project I solved several tasks connected to path planning in order to find the optimal path for a robot to reach a goal node from a starting node.

I wrote a method to find the closest node to an input point. This is later used to find the start and goal nodes. Then, I implemented the A* search algorithm. Next, I generated the optimal path found by the A* algorithm. Then, I smoothed this path so that a robot can follow it easier. In the end I also completed the method to generate a probabilistic roadmap so that it can be used instead of a grid map.

Demonstration Video Link: https://youtu.be/Pa7BX4TIEh0

## Heuristic weight:

Search algorithm with heuristic weight of 0 (Dijkstra`s)

![grafik](https://user-images.githubusercontent.com/115760050/202973053-df97c882-6e54-49a2-9c52-9d1c21021b1d.png)


Search algorithm with heuristic weight of 1 (A*)

![grafik](https://user-images.githubusercontent.com/115760050/202973205-70ac0d6e-a979-429d-a10b-8b6dcebdf26c.png)


## Smoothing:

Alpha influences how strong the original location shall influence the calculation of the new point. Beta influences how strong the distance to the previous and the next point shall influence the calculation of the new point. High alpha leads to less smoothing, high beta leads to more smoothing.

alpha =0,01; beta = 0,2

![grafik](https://user-images.githubusercontent.com/115760050/202973355-0f6ec025-585e-46ea-b461-5cbc22cdd20c.png)

alpha = 0,2; beta = 0,01

![grafik](https://user-images.githubusercontent.com/115760050/202973390-642086ea-0859-471f-8285-0268231fb71f.png)



