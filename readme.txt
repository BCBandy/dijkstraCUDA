Project was build using cuda 6.5

Program builds a 3d maze and finds shortest path to the end using dijkstra's algorithm.
Two versions of dijkstra's algorithm are used, CPU and GPU.
Both version's times are recorded and compared in output

Input description:
x y z is dimension of maze
rand is % of walls/floors that are open [1-10], 0 = no extra walls/floors, 10 all walls/floors open.

input:
x y z rand threads