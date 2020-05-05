# Simple ROS code generator
This is the repository of simple XText based plugin for Eclipse. Our goal is to create and describe interface bolerplate code and generate ROS/ROS2 compatible code (Python/C++).

You can describe the following for each node:
- Publisher/Subscriber interfaces: it will be generated as an abstract class.
- Parameters: default YAML file will be generated alongside a function which loads these parameters.
- Message definition for custom files: WIP
- RPC: WIP
- QOS (for ROS2): WIP

We also propose behavior description which can be assigned to a node.
This toolset is under development and in very early phase. See the wiki for installation instructions.
