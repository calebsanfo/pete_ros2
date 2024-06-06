# gps_publisher_pkg

## Installation

1. Build the package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select gps_publisher_pkg
    ```

2. Source the setup file:
    ```sh
    source install/setup.bash
    ```

3. Install Python dependencies:
    ```sh
    ./src/gps_publisher_pkg/install_dependencies.sh
    ```

4. Run the node:
    ```sh
    ros2 run gps_publisher_pkg gps_publisher
    ```

## Dependencies

This package requires the `adafruit-circuitpython-gps` Python library. The dependencies are managed via `pip` and listed in the `requirements.txt` file.
