# Development Workflow

## 1. Start Container
```bash
cd -RoboGuide
docker compose up -d
```

---

## 2. Write Your Code

Write your Python node in the appropriate package:

**For vision/detection:**
```bash
cd -RoboGuide/ros2_ws/src/amr_perception/amr_perception
```

**For navigation/planning:**
```bash
cd -RoboGuide/ros2_ws/src/amr_navigation/amr_navigation
```

**For simulation/worlds:**
```bash
cd -RoboGuide/ros2_ws/src/amr_gazebo/
# Add to urdf/, worlds/, or launch/ folders
```

---

## 3. Register Your Node

**Only needed for Python nodes in amr_perception or amr_navigation**

Edit the package's `setup.py`:
```bash
nano -RoboGuide/ros2_ws/src/amr_perception/setup.py
```

Add your node to `entry_points`:
```python
entry_points={
    'console_scripts': [
        'your_node = amr_perception.your_file:main',
    ],
},
```

---

## 4. Build Code

**Enter container:**
```bash
docker exec -it amr_dev bash
```

**Build:**
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## 5. Test Your Node

**Test individually:**
```bash
ros2 run amr_perception your_node
```

---

## 6. Run Full Simulation

**Launch everything together:**
```bash
ros2 launch amr_gazebo hospital_demo.launch.py
```

---

## 7. Push to GitHub

**Exit container:**
```bash
exit
```

**Push:**
```bash
cd -RoboGuide
git add .
git commit -m "Add [your feature]"
git push origin your-branch-name
```

---

## Stop Container
```bash
docker compose down
```

---

## Common Commands

**Open multiple terminals (for running multiple nodes):**
```bash
docker exec -it amr_dev bash
```

**List running nodes:**
```bash
ros2 node list
```

**Check topics:**
```bash
ros2 topic list
ros2 topic echo /topic_name
```
