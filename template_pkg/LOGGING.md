# ROS Logging Configuration

This package supports configurable ROS logging levels following ROS2 best practices.

## Configuration Options

The `log_level` parameter can be set to control the verbosity of logging output:

- `debug` - Most verbose, shows all debug messages
- `info` - Default level, shows informational messages and above
- `warn` - Shows warnings and errors
- `error` - Shows only errors and fatal messages
- `fatal` - Shows only fatal errors

## Usage

### Via YAML Configuration File

Edit the `config/minimal_publisher.yaml` file:

```yaml
minimal_publisher:
    ros__parameters:
        log_level: "debug"  # Options: debug, info, warn, error, fatal
```

### Via Launch File

Pass the log level as a launch argument:

```bash
ros2 launch template_pkg minimal_publisher.launch.py log_level:=debug
```

### Via Command Line Parameter

When running the node directly:

```bash
ros2 run template_pkg minimal_publisher --ros-args -p log_level:=debug
```

## Example Output

With `log_level: debug`, you will see additional debug messages:

```
[INFO] [minimal_publisher]: Logger level set to: debug
[DEBUG] [minimal_publisher]: Timer callback triggered, count: 1
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
[DEBUG] [minimal_publisher]: Timer callback triggered, count: 2
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
```

With `log_level: warn`, only warnings and errors will be shown.
