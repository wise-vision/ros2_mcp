
# WiseVision ROS 2 MCP Server

[![Discord](https://img.shields.io/badge/Discord-Join%20Us-5865F2?logo=discord)](https://discord.gg/9aSw6HbUaw)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-purple)
[![Docker](https://img.shields.io/badge/Docker-MCP-blue?logo=docker)](https://hub.docker.com/mcp/server/ros2/overview)
[![GitHub stars](https://img.shields.io/github/stars/wise-vision/mcp_server_ros_2?style=social)](https://github.com/wise-vision/mcp_server_ros_2/stargazers)

![Flow graph](docs/assets/flow-graph.gif)

A **Python** implementation of the **Model Context Protocol (MCP)** for **ROS 2**. This server enables AI tooling to connect with **ROS 2** nodes, topics, and services using the **MCP** standard over **stdio**. Designed to be **the easiest** **ROS 2** MCP server to configure.

## 🎯 Why Choose This MCP Server?

**Save hours of development time** with native AI integration for your ROS 2 projects:

## Why this ROS 2 MCP server ⭐

- **⚡ 1-minute setup** - World's easiest ROS 2 MCP configuration
- **0️⃣ Zero-friction setup** - stdio transport, no brokers, no webserver.
- **🔌 Auto type discovery** - a built-in “list interfaces” tool dynamically enumerates available topics and services together with their message/service definitions (fields, types, schema) — so the client always knows exactly what data can be published or called.
- **✨ Nested field support**: Handle complex message structures with ease.
- **🤖 AI-powered debugging** - Let AI help you troubleshoot ROS 2 issues in real time
- **📊 Smart data analysis** - Query your robot's sensor data using natural language
- **🚀 Boost productivity** - Control robots, analyze logs, and debug issues through AI chat
- **💡 No ROS 2 expertise required** - AI translates your requests into proper ROS 2 commands
- **🐋 Dockerized**: Ready-to-use Docker image for quick deployment.
- **🔧 Auto QoS selection**: Automatically selects appropriate Quality of Service settings for topics and services, ensuring optimal communication performance without manual configuration.

**Perfect for:** Robotics developers, researchers, students, and anyone working with ROS 2 who wants to leverage AI for faster development and debugging.

If you find this useful, please ⭐ star the repo — it helps others discover it.

🚀 **Enjoying this project?**  
Feel free to contribute or reach out for support! Write issues, submit PRs, or join our [Discord community](https://discord.gg/9aSw6HbUaw) to connect with other ROS 2 and AI enthusiasts.


# 🌍 Real-world examples:
![Demo](docs/assets/mcp-ros2-server.gif)

# ✨ Features
- List available topics 
- List available services 
- Call services
- Subscribe to topics to collect messages
- Publish messages to topics
- Echo messages on topics
- Get fields from message types
- Get messages from [WiseVision Data Black Box](https://github.com/wise-vision/wisevision_data_black_box) ([InfluxDB](https://www.influxdata.com) alternative to [Rosbag2](https://github.com/ros2/rosbag2))


**Note:** To call a service with a custom (non-default) type, source the package that defines it before starting the server.


# ⚙️ Installation

Follow the [installation guide](installation/README.md) for step-by-step instructions:
- [🧩 Install in Visual Studio Code Copilot](installation/README.md#configure-visual-studio-code-copilot)
- [🤖 Install in Claude Desktop](installation/README.md#configure-claude-desktop)
- [💻 Install in Warp](installation/README.md#configure-warp)
- [🐳 Build Docker Image locally](installation/README.md#build-docker-image-locally)



### 🔧 ROS 2 Tools

#### 📋 **Topics**
| Tool | Description | Inputs | Outputs |
|------|-------------|--------|---------|
| **`ros2_topic_list`** | Returns list of available topics | – | `topic_name` (string): Topic name <br> `topic_type` (string): Message type |
| **`ros2_topic_subscribe`** | Subscribes to a ROS 2 topic and collects messages for a duration or message limit | `topic_name` (string) <br> `duration` (float) <br> `message_limit` (int) <br> *(defaults: first msg, 5s)* | `messages` <br> `count` <br> `duration` |
| **`ros2_get_messages`** | Retrieves past messages from a topic (data black box) | `topic_name` (string) <br> `message_type` (string) <br> `number_of_messages` (int) <br> `time_start` (str) <br> `time_end` (str) | `timestamps` <br> `messages` |
| **`ros2_get_message_fields`** | Gets field names and types for a message type | `message_type` (string) | Field names + types |
| **`ros2_topic_publish`** | Publishes message to a topic | `topic_name` (string) <br> `message_type` (string) <br> `data` (dict) | `status` |

---

#### 🛠 **Services**
| Tool | Description | Inputs | Outputs |
|------|-------------|--------|---------|
| **`ros2_service_list`** | Returns list of available services | – | `service_name` (string) <br> `service_type` (string) <br> `request_fields` (array) |
| **`ros2_service_call`** | Calls a ROS 2 service | `service_name` (string) <br> `service_type` (string) <br> `fields` (array) <br> `force_call` (bool, default: false) | `result` (string) <br> `error` (string, if any) |


# 🐞 Debugging

Since MCP servers run over stdio, debugging can be challenging. For the best debugging
experience, we strongly recommend using the [MCP Inspector](https://github.com/modelcontextprotocol/inspector).

You can launch the MCP Inspector via [ `npm` ](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) with this command:

```bash
npx @modelcontextprotocol/inspector uv --directory /path/to/mcp_server_ros2 run mcp_ros_2_server
```

Upon launching, the Inspector will display a URL that you can access in your browser to begin debugging.

## 📚 Origins and evolution

We built this server to make AI‑assisted ROS 2 development fast and reliable. Internally, we needed a simple way for agents to discover message types, publish/subscribe to topics, and call services—without boilerplate or flaky networking. That led to a few core design goals:
- Handle all ROS 2 message types (including nested fields) so agents can write and test any code
- Integrate topic pub/sub and service calls to validate behavior end‑to‑end
- Work seamlessly with GitHub Copilot in VS Code and other MCP clients
- Use a simple stdio transport to avoid network complexity

After dogfooding it, we open‑sourced the project to help the broader ROS 2 community build faster with AI. It’s now useful not only for development, but also for controlling robots, running QoS experiments, and analyzing live data and robot/swarm state. The project is actively maintained—features and improvements ship regularly based on user feedback. If this project helps you, please star the repo and share your use case!
