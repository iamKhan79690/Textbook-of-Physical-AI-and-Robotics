# Data Model: Overview Chapter - Key Concepts

## Entities & Relationships

### 1. Physical AI
- **Description**: A synergistic partnership between people, agents, and robots, enabling intelligent interaction with the physical world.
- **Attributes**:
    - **People**: Human involvement in design, oversight, and collaboration.
    - **Agents**: Software entities that process information, make decisions, and control robots.
    - **Robots**: Physical machines that interact with the environment based on agent instructions.
- **Relationships**:
    - `Physical AI` `HAS` `People`, `Agents`, `Robots`
    - `Agents` `CONTROL` `Robots`
    - `People` `GUIDE` `Agents`

### 2. Embodied Intelligence
- **Description**: A type of AI that understands and operates within the constraints of physical laws, moving beyond purely digital environments.
- **Attributes**:
    - **Understanding Physical Laws**: The ability to comprehend and predict real-world physics.
    - **Real-world Interaction**: Direct engagement with the physical environment.
- **Relationships**:
    - `Embodied Intelligence` `IS A TYPE OF` `AI`
    - `Embodied Intelligence` `REQUIRES` `Physical AI` (for practical application)

### 3. The Three Heavy Loads
- **Description**: The computationally intensive demands that define the hardware requirements for Physical AI systems.
- **Components**:
    - **Physics Simulation**: Recreating real-world physics for training and testing.
    - **Perception**: Processing sensory data (e.g., vision, lidar) from the physical environment.
    - **Generative AI**: Using AI models to create behaviors, plans, or content for robots/agents.
- **Relationships**:
    - `Physical AI` `DEPENDS ON` `The Three Heavy Loads`

### 4. Digital Twin Workstation
- **Description**: A high-performance computing setup designed for developing, simulating, and training Physical AI models, particularly for creating digital twins.
- **Key Specifications**:
    - **CPU**: (Implicitly high-performance, for general computation and simulation)
    - **RAM**: 64GB
    - **OS**: Ubuntu 22.04
    - **GPU**: RTX 4070 Ti+ (or equivalent, for GenAI and simulation)
- **Relationships**:
    - `Digital Twin Workstation` `SUPPORTS` `The Three Heavy Loads`
    - `Digital Twin Workstation` `IS A PREREQUISITE FOR` `Physical AI Development`

### 5. Edge Kit
- **Description**: A compact, low-power computing device for deploying Physical AI solutions directly into real-world robots or edge applications.
- **Key Specifications**:
    - **Compute Module**: Jetson Orin Nano
    - **Perception Sensor**: RealSense (or similar depth/vision sensor)
- **Relationships**:
    - `Edge Kit` `DEPLOYS` `Physical AI Solutions`
    - `Edge Kit` `UTILIZES` `Perception`

### 6. Course Roadmap Concepts
- **Description**: Key technologies and methodologies covered in the book, outlining the learning progression.
- **Components**:
    - **ROS 2 (Robot Operating System 2)**: Foundation for robot control and communication.
    - **VLA (Vision-Language-Action Models)**: Advanced AI models for robot decision-making based on visual and linguistic input.
- **Relationships**:
    - `Course Roadmap` `PROGRESSES FROM` `ROS 2` `TO` `VLA`

### 7. Latency in Robot Control vs. Cloud Gaming
- **Description**: An explanation of why the latency requirements for real-time robot control are fundamentally different and more stringent than those for cloud gaming.
- **Key Differences**:
    - **Robot Control**: Requires sub-millisecond, highly deterministic response times for safe and effective physical interaction.
    - **Cloud Gaming**: Tolerates higher, more variable latency as it primarily affects user experience, not physical safety or success.
- **Relationships**:
    - `Latency` `IMPACTS` `Robot Control` `MORE CRITICALLY THAN` `Cloud Gaming`

