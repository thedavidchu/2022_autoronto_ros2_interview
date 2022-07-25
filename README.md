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

The Solution node's publishing to "/solution" is 1 cycle behind the Test node's publishing to "/input" and "/target". We could have potentially used a "global timer" by running the publishing to "/solution" once we have received valid input from both "/input" and "/target". This would potentially require thread synchronization (which I believe could be done with the regular `input_is_valid_` and `target_is_valid_` variables; we do not need atomic operations, because we are setting first and then checking the other. No matter which way we interleave the sets-and-checks, it works). I did not end up going with this option because it is not as robust at publishing once per second.

```
Thread 1 ("/input") | Thread 2 ("/target")
--------------------|---------------------
set input_is_valid_ |
                    | set target_is_valid_
check target_is_valid_ (yes) => run publisher and reset |
                    | check target_is_valid_ (must be no)
                    
=> the checking the other is valid must be in a critical region (bc we don't want to publish twice)!
```


The members: `input_is_valid_` and `target_is_valid_` in the Solution class are unnecessary. I added them for redundant error checking. The could be used in the solution above as a means of error checking.
