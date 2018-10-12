# py_test_repo
Launch with
```
ros2 run py_test_repo test
```

then test with :
```
ros2 topic pub -1 /py_test_repo std_msgs/Empty "{}"
```

You'll see the blip but will get stuck on spin_once() and will never receive the blop.