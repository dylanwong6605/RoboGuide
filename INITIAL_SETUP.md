# Initial Setup - Do This Once Per Machine

## 1. Install Docker
```bash
sudo apt-get update
sudo apt-get install -y curl wget git

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
rm get-docker.sh

sudo apt-get install -y docker-compose-plugin
```

**IMPORTANT: Log out and log back in**

Verify:
```bash
docker --version
docker compose version
```

---

## 2. Clone Repository
```bash
git clone https://github.com/dylanwong6605/-RoboGuide.git
cd -RoboGuide
```

---

## 3. Allow GUI Applications
```bash
xhost +local:docker
echo "xhost +local:docker" >> ~/.bashrc
```

---

## 4. Build Docker Container
```bash
docker compose build
```

**This takes 10-30 minutes** (downloads ROS 2, Gazebo, PyTorch, YOLO)

---

## 5. Test Container Works
```bash
docker compose up -d
docker exec -it amr_dev bash
```

You should see: `ROS 2 Humble environment loaded`

Type `exit` to leave container.

---

## Done! ✅
