# Airport Delay Path Finder

A C++ application that finds optimal flight paths between US airports using Dijkstra's and A* algorithms.

## How to Run

### Prerequisites
- C++ compiler (g++ with C++20 support)

### Build and Run
```bash
# Compile the project
g++ -std=c++20 -o prog_3b_final src/*.cpp -I src/

# Run the program
./prog_3b_final
```

### Usage
The program provides an interactive menu:
1. **Find shortest path** - Enter airport codes and choose algorithm (Dijkstra or A*)
2. **List airports by state** - Enter 2-letter state code (e.g., CA)
0. **Exit** - Quit the program

### Example
```
==== Airport Delay Path Finder ====
1. Find shortest path
2. List airports by state
0. Exit
Choice: 1

Enter starting airport code: ATL
Enter destination airport code: LAX
Choose algorithm:
1. Dijkstra
2. A*
Enter choice: 1
```

## Data
Uses real US flight data from `data/flights_real_usa.csv` with 100,000+ flight records. 
