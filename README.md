# GPT Codex
**Simplifying code with AI-driven guidance**


Welcome to the **GPT Codex** repository! This repo demonstrates how GPT-style tooling can assist with code generation and repository management.

## Features

- Minimal example project
- Simple structure for quick experimentation
- Markdown-based documentation

## Getting Started

Clone the repository:

```bash
git clone <repo-url>
```

Feel free to explore and modify the contents.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](LICENSE) (placeholder)

## Engineering Calculator ROS2 Package

This repository contains a ROS 2 (Foxy) package called `engineering_calculator`.
The package provides a service named `calculate_expression` that evaluates
mathematical expressions using a safe parser. The service returns the computed
result or an error message.

### Building and Running

1. Install ROS 2 Foxy and source your ROS 2 environment.
2. Build the package using `colcon build`:

```bash
colcon build --packages-select engineering_calculator
```

3. Source the workspace and run the service node:

```bash
. install/setup.bash
ros2 run engineering_calculator calculator_server
```

### Testing

The expression evaluator can be tested without ROS 2 using `pytest`:

```bash
pytest engineering_calculator/test
```

