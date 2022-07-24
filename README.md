# aUToronto ROS2 Interview

My solution uses a Node called "solution", which takes a vector of int8's from "/input" and an int8 target from "/target" and publishes the indices of numbers in the input such that they add to the target. The result is published on "/solution".

To run, open two terminals:

```bash
# Terminal 1: in autoronto_interview folder
./run.sh
```

```bash
# Terminal 2: in autoronto_interview folder
./run.sh solution
```

## Notes

The must not be another target called "davidchus_test". This will collide with my own test.