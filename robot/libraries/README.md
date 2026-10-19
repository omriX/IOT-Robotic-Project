# libraries/micro_ros_arduino — custom build

This is not the stock Library Manager version. 
If it's swapped back, `/odom` and ROS parameters
silently break - rebuild from source instead.

| Setting | Stock | Here | Why |
|---|---|---|---|
| `RMW_UXRCE_MAX_SERVICES` | 1 | 8 | parameter server needs ~6 services |
| `UCLIENT_CUSTOM_TRANSPORT_MTU` | 512 | 2048 | `/odom` (~720 B) is over the 512 B stock MTU |

Also needs to be copied to Arduino's actual library
folder - that's what the toolchain reads. Keep both in sync.

**Rebuild:** clone `micro_ros_arduino` (humble branch), drop in
`micro_ros_arduino/extras/library_generation/colcon.meta` from here, then:

```bash
docker run --rm -v <build_dir>:/project --env MICROROS_LIBRARY_FOLDER=extras \
  microros/micro_ros_static_library_builder:humble -p esp32
```
