# A* Algorithm Visualization
This project implements A-star algorithm to find the shortest path between two nodes in a graph. It includes visualization of the algorithm's execution and can optionally generate a video of the process.


## Description
This Python script implements A-star algorithm to find the shortest path between two nodes in a graph defined by input.txt and coords.txt. The script generates visualizations of each step and can create a video showcasing the algorithm's progress.

## Features
- **Graph Input Processing**: Reads graph edges and nodes from `input.txt` and node coordinates from `coords.txt`.
- **Shortest** Path Calculation: Efficiently computes the shortest path using a priority queue.
- **Visualization**: Generates images at each step to visualize the search process.
- **Video Generation**: Optionally creates a video from the generated images using OpenCV.
- **Command-Line Interface**: Control video generation and frame rate via command-line arguments.
- **Output**: Writes the shortest path and distances to a text file named `<SJSU_ID>.txt.`

## Prerequisites
- Python `3.10` (`3.10.5` preferred)
- matplotlib
- opencv-python
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

### Command-Line Arguments
- `--video`: Set to 1 to generate a video, 0 otherwise. Default is 1.
- `--steps_per_frame`: Determines how many algorithm steps are included in one video frame. Increasing this value will speed up the video by including more steps per frame. Default is 3.

### Example Commands
- Generate Video (with default frame rate):
```bash
python main.py --video 1
```

- Do Not Generate Video:
```bash
python main.py --video 0
```

- Generate Video with Custom Steps Per Frame:
```bash
python main.py --video 1 --steps_per_frame 5
```
