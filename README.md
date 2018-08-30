# test_repo
Launch with
```
ros2 run test_repo test
```

then test with :
```
ros2 topic pub -1 /test_repo std_msgs/Empty "{}"
```

You'll see the blip but will get stuck on spin_once() and will never receive the blop.