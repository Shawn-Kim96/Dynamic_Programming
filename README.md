# Dynamic Programming Shortest Path Finder

This project implements a Dynamic Programming (DP) algorithm to find the shortest path between two nodes in a graph. The algorithm leverages the graph structure from input files and outputs the optimal path and solution in a text file.


## Description
This Python script implements a shortest path algorithm using Dynamic Programming. It reads input data from `input.txt` (for graph edges and nodes) and `coords.txt` (for node coordinates). The script calculates the optimal path between specified start and end nodes, and the results are written to `<SJSU_ID>.txt`.

## Features
- **Graph Input Processing**: Reads graph edges and nodes from `input.txt` and coordinates from `coords.txt`.
- **Shortest Path Calculation**: Computes the shortest path using a deque for cost-to-come.
- **Output**: Writes the shortest path and distances to `<SJSU_ID>.txt`, where the filename includes the user's SJSU ID.

## Prerequisites
- Python `3.10` (`3.10.5` preferred)
- tqdm

## Installation
You can set up the Python environment using either requirements.txt with pip or using Poetry.

### Using requirements.txt
1. Clone the Repository (if applicable) or navigate to the project directory.
2. Create a Virtual Environment (optional but recommended):
```bash
python -m venv venv
```

3. Activate the Virtual Environment:
- On Windows:
```bash
venv\Scripts\activate
```
- On macOS/Linux:
```bash
source venv/bin/activate
```

4. Install the Dependencies:
```bash
pip install -r requirements.txt
```


### Using Poetry
1. Install Poetry (if not already installed):
```bash
pip install poetry
```
2. Clone the Repository (if applicable) or navigate to the project directory.
3. Install the Dependencies:
```bash
poetry install
```
4. Activate the Poetry Shell:
```bash
poetry shell
```

## Usage
Ensure that input.txt and coords.txt are in the same directory as the script.

Run the script to caclutate the shortest path and generate output.
```bash
python 018219422.py
```

## Input Format
- `input.txt`:
    - First three lines: Total nodes, start node, and end node.
    - Remaining lines: Edge definitions in the format start_node end_node distance.
- `coords.txt`: Coordinates for each node in the format x y.


## Output
Results are saved to `<SJSU_ID>.txt`:

- The first line lists the optimal path.
- The second line lists optimal cost-to-come values for all nodes. (writen in list)