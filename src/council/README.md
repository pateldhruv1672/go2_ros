# Council Multi-Agent Navigation System for Unitree Go2

A LangChain-based multi-agent architecture for autonomous navigation of the Unitree Go2 robot.

## Overview

The Council system uses a "council of agents" approach where specialized agents analyze different sensor modalities and collaboratively decide on navigation actions:

```
┌─────────────────────────────────────────────────────────────────┐
│                     COUNCIL ORCHESTRATOR                        │
│  (Voting, Consensus, Conflict Resolution, Meta-Agent Review)    │
└─────────────────────────────────────────────────────────────────┘
        ▲               ▲               ▲               ▲
        │               │               │               │
┌───────┴───────┐ ┌─────┴─────┐ ┌───────┴───────┐ ┌─────┴─────┐
│   IMU AGENT   │ │  CAMERA   │ │  LIDAR AGENT  │ │   SLAM    │
│               │ │   AGENT   │ │               │ │   AGENT   │
│ - Stability   │ │ - Scene   │ │ - 3D Spatial  │ │ - Mapping │
│ - Orientation │ │ - Objects │ │ - Obstacles   │ │ - Localize│
│ - Motion      │ │ - Hazards │ │ - Free Space  │ │ - Plan    │
└───────────────┘ └───────────┘ └───────────────┘ └───────────┘
        ▲               ▲               ▲               ▲
        │               │               │               │
┌─────────────────────────────────────────────────────────────────┐
│                        SENSOR HUB                                │
│  (ROS2 Subscriptions: IMU, Camera, LiDAR, Odometry)             │
└─────────────────────────────────────────────────────────────────┘
```

## Agents

### 1. IMU Agent
- Analyzes accelerometer, gyroscope, and orientation data
- Detects stability issues, tilting, or falling
- Provides motion state assessment (stationary, moving, unstable)

### 2. Camera Agent
- Uses Vision Language Models (VLM) to understand the scene
- Detects obstacles, people, and task-relevant objects
- Identifies navigable paths and hazards

### 3. LiDAR Agent
- Processes 3D point cloud data
- Provides precise distance measurements in all directions
- Identifies clear paths and closest obstacles

### 4. SLAM Agent
- Builds and maintains an occupancy grid map
- Tracks robot position using odometry and scan matching
- Provides exploration guidance and path planning

## Installation

### Prerequisites
- ROS2 Humble or later
- Python 3.8+
- Unitree Go2 with ROS2 SDK

### Install Dependencies

```bash
cd /home/dkp/ros2_ws/src/council
pip install -r requirements.txt
```

### Build the Package

```bash
cd /home/dkp/ros2_ws
colcon build --packages-select council
source install/setup.bash
```

## Configuration

### Environment Variables

```bash
# LLM API Key (OpenRouter recommended)
export OPENROUTER_API_KEY="your-api-key"

# Or use OpenAI directly
export OPENAI_API_KEY="your-api-key"

# Optional: Override ROS2 topics
export COUNCIL_CMD_VEL_TOPIC="/cmd_vel"
export COUNCIL_CAMERA_TOPIC="/camera/color/image_raw"
export COUNCIL_LIDAR_TOPIC="/lidar/points"
export COUNCIL_DEBUG="true"
```

### Default ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_joy` | geometry_msgs/Twist | Velocity commands |
| `/camera/image_raw` | sensor_msgs/Image | Camera feed |
| `/point_cloud2` | sensor_msgs/PointCloud2 | LiDAR data |
| `/imu/data` | sensor_msgs/Imu | IMU readings |
| `/odom` | nav_msgs/Odometry | Odometry data |

## Usage

### Direct Execution

```bash
cd /home/dkp/ros2_ws/src/council
source /home/dkp/ros2_ws/install/setup.bash
python3 -m council.main
```

### Using ROS2 Run

```bash
ros2 run council council_node
```

### Using Launch File

```bash
ros2 launch council council.launch.py \
    camera_topic:=/camera/image_raw \
    lidar_topic:=/point_cloud2 \
    debug:=true
```

## Controls

| Key | Action |
|-----|--------|
| `m` | Toggle MANUAL/AI mode |
| `t` | Set navigation task |
| `SPACE` | Emergency stop |
| `q` | Quit |
| `i` | Forward |
| `k` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `J` | Strafe left |
| `L` | Strafe right |

## Example Tasks

```
Enter Task: Find the door and navigate to it
Enter Task: Explore the room and map the environment
Enter Task: Follow the person in front of you
Enter Task: Navigate to the red chair
Enter Task: Avoid all obstacles and reach the window
```

## Voting Strategies

The orchestrator supports multiple strategies for combining agent recommendations:

- **WEIGHTED** (default): Confidence-weighted voting with safety prioritization
- **SAFEST**: Always choose the safest action
- **MAJORITY**: Simple majority voting
- **CONSENSUS**: Require agreement, stop on conflict

## Architecture

```
council/
├── __init__.py
├── config.py              # Configuration management
├── main.py                # Main entry point
├── orchestrator.py        # Council orchestrator
├── navigation_planner.py  # Path planning
├── agents/
│   ├── __init__.py
│   ├── base_agent.py      # Base agent class
│   ├── imu_agent.py       # IMU analysis
│   ├── camera_agent.py    # Vision analysis
│   ├── lidar_agent.py     # LiDAR analysis
│   └── slam_agent.py      # SLAM
├── ros_interfaces/
│   ├── __init__.py
│   └── sensor_hub.py      # ROS2 sensor subscriptions
├── launch/
│   └── council.launch.py
├── resource/
│   └── council
├── requirements.txt
├── package.xml
├── setup.py
└── setup.cfg
```

## Extending the System

### Adding a New Agent

1. Create a new agent file in `agents/`:

```python
from .base_agent import BaseAgent, AgentResponse

class MyAgent(BaseAgent):
    def __init__(self, **kwargs):
        super().__init__(name="MyAgent", **kwargs)
    
    def get_system_prompt(self) -> str:
        return "Your system prompt..."
    
    def prepare_input(self, sensor_data, task):
        # Format input for LLM
        return [HumanMessage(content="...")]
    
    def parse_response(self, response):
        # Parse LLM output
        return AgentResponse(...)
```

2. Register in `orchestrator.py`:

```python
self.agents["my_agent"] = MyAgent(debug=self.debug)
```

### Custom Voting Strategy

Implement a new method in `CouncilOrchestrator`:

```python
def _my_voting_strategy(self, responses):
    # Your voting logic
    return CouncilDecision(...)
```

## Troubleshooting

### No Camera Data
- Check topic: `ros2 topic list | grep camera`
- Echo topic: `ros2 topic echo /camera/image_raw --no-arr`

### LLM Timeout
- Verify API key: `echo $OPENROUTER_API_KEY`
- Check network connectivity
- Try a different model in `config.py`

### Robot Not Moving
- Check E-STOP (press SPACE to toggle)
- Verify cmd_vel topic matches robot's expected topic
- Check for high-risk detections stopping movement

## License

MIT
