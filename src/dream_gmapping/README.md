# Dream Gmapping

## Tools

- Record Bags

```bash
rosbag record --bz2 /dream/scan /dream/wheel_pos /tf
```

## Assumptions

1. Since the number of lidar points could vary from brand to brand, model to model, and even from frequency to frequency,
   it is determined dynamically when the first laser scan is received.
