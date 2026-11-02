# Setup guide: from a Windows laptop to a working simulation

This is the **tested** path that worked for this project, with every problem we hit and its fix. It follows the official course guide ([`course_material/install_Guide.docx`](course_material/install_Guide.docx)) and adds the missing pieces.

**Target stack:** Ubuntu **22.04** · ROS2 **Humble** · Gazebo **Harmonic** (gz-sim 8) · ArduPilot SITL (ArduCopter) · `ardupilot_gazebo` plugin · QGroundControl 5.0.8 · Python 3 with `pymavlink` and `matplotlib`.

**Machine used:** Lenovo Legion Y540 (81SX), i7-9750H (6 cores, 12 threads), 24 GB RAM, NVIDIA GTX 1660 Ti, Windows 11.

---

## 0. Which way to run Ubuntu (decision record)
| Option | Result | Verdict |
|---|---|---|
| **WSL2** | ROS2 installed fine, but WSL had **no GPU**: `/usr/lib/wsl/lib` had no NVIDIA libraries, `nvidia-smi` was missing, and `glxinfo` showed `llvmpipe`, meaning software rendering. Also, `wsl --install` gave Ubuntu **26.04** by default, which is the wrong version. | Abandoned |
| Dual boot | Would work best, but needs BIOS/AHCI changes and BitLocker handling (a `safeboot` flag was set and later removed). | Not needed |
| **VMware Workstation Pro + Ubuntu 22.04 VM** | 3D works: `glxinfo` shows `SVGA3D`, OpenGL **4.3**. Everything runs. | **Used** |
| VirtualBox | 3D acceleration is too weak for Gazebo | Avoid |

Ubuntu **must be 22.04** (Jammy). ROS2 Humble and the Humble–Harmonic bridge only exist for 22.04, and on 26.04 none of the guide works. Check with `lsb_release -d`.

---

## 1. Create the VM (on Windows)
1. **Install VMware Workstation Pro.** It's free for personal use and downloaded from Broadcom's support portal (needs an account).
2. **Make VMware use the NVIDIA GPU.** In NVIDIA Control Panel, open **Manage 3D settings → Program Settings → Add**, choose `C:\Program Files (x86)\VMware\VMware Workstation\x64\vmware-vmx.exe`, and set it to **High-performance NVIDIA processor**.
3. **Download the Ubuntu installer:** `ubuntu-22.04.5-desktop-amd64.iso` from https://releases.ubuntu.com/22.04/. "amd64" is correct for Intel CPUs too; don't take `live-server` or `wsl`.
4. **Create the VM:** VMware → New Virtual Machine → Typical → choose the ISO.
   - **Memory:** 13 GB
   - **Processors:** 8 (8 × 1 core is fine)
   - **Disk:** 100–120 GB
   - **Display:** tick **Accelerate 3D graphics**, graphics memory at the maximum
   - **Virtualization engine:** leave all boxes **unticked**. With Hyper-V/WSL present they can stop the VM from starting.
   - **Network adapter:** NAT, with **Connect at power on** ticked
5. **Install Ubuntu:** Normal installation, tick **Install third-party software**, then **Erase disk and install Ubuntu**. This only erases the VM's own virtual disk.

## 2. First boot: network and software sources
**Problem we hit:** `apt` said *"Package 'curl' has no installation candidate"* and *"Unable to locate package git"*. Ubuntu had been installed without internet, so its software sources pointed only to the install disc.

**2.1 Check the internet first:**
```bash
ping -c 3 google.com
```
If that fails:
1. In VMware, open **VM → Settings → Network Adapter**, and make sure **NAT** and **Connect at power on** are ticked. "Connected" is greyed out while the VM is off; that's normal.
2. On Windows, open **VMware → Edit → Virtual Network Editor → Change Settings → Restore Defaults**.
3. Run `services.msc` and make sure **VMware NAT Service** and **VMware DHCP Service** are Running.
4. Alternatively, switch the adapter to **Bridged** with **Replicate physical network connection state** ticked.
5. Back in Ubuntu, restart its network:
```bash
sudo systemctl restart NetworkManager
```

**2.2 Point Ubuntu at the online package sources** (or use Software & Updates → tick main/universe/restricted/multiverse → Main server → untick CD-ROM):
```bash
sudo tee /etc/apt/sources.list > /dev/null <<'EOF'
deb http://archive.ubuntu.com/ubuntu jammy main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu jammy-updates main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu jammy-backports main restricted universe multiverse
deb http://security.ubuntu.com/ubuntu jammy-security main restricted universe multiverse
EOF
```
```bash
sudo apt update && sudo apt upgrade -y
```

**2.3 Install the base tools and VMware integration, then reboot:**
```bash
sudo apt install open-vm-tools-desktop mesa-utils git curl -y
```
```bash
sudo reboot
```

**2.4 Check the 3D acceleration:**
```bash
glxinfo -B | grep -iE "renderer string|version string"
```
You want `SVGA3D` and OpenGL **4.3**. If it shows `llvmpipe`, there's no 3D: shut down the VM and tick **Accelerate 3D graphics**. `Accelerated: no` may still be printed with SVGA3D; that's a reporting quirk.

## 3. ROS2 Humble (course guide Step 2)
```bash
sudo apt install locales software-properties-common -y && sudo locale-gen en_US en_US.UTF-8
```
```bash
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8
```
```bash
sudo add-apt-repository universe -y
```
```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
```
```bash
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
```
```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
```
```bash
sudo apt update && sudo apt upgrade -y
```
```bash
sudo apt install ros-humble-desktop ros-dev-tools -y
```
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
**Check:** `ros2 run demo_nodes_cpp talker` should print `Publishing: 'Hello World: 1'` and keep counting. ✅ Verified.

## 4. Gazebo Harmonic + ROS bridge (course guide Step 3)
```bash
sudo apt-get update && sudo apt-get install curl lsb-release gnupg -y
```
```bash
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
```
```bash
sudo apt-get update && sudo apt-get install gz-harmonic -y
```
```bash
sudo apt install ros-humble-ros-gzharmonic -y
```
**Problem we hit:** `E: Unable to locate package gz-harmonic`. The OSRF key or source line hadn't been written. Re-run the `curl` key command and the `echo ... | sudo tee` line (it must print `... ubuntu-stable jammy main`), then `sudo apt-get update` again. The update output must show `packages.osrfoundation.org`.

**Check:** `gz sim --version` should report version 8.x.

**Problem we hit:** in the VM, the Gazebo window **flickered or blinked**. Fix: **always add `--render-engine ogre`**:
```bash
gz sim shapes.sdf --render-engine ogre
```
✅ This works. If it still misbehaves, try `LIBGL_DRI3_DISABLE=1`, or run Gazebo without a window using `gz sim -s ...`.

Note: `gz sim iris_runway.sdf` fails with *"Unable to find or download file"* until Step 6 is done. That's expected.

## 5. ArduPilot SITL (course guide Step 4)
```bash
cd ~ && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
```
If the network drops during the clone, resume it with:
```bash
cd ~/ardupilot && git submodule update --init --recursive
```
Install ArduPilot's dependencies (10–20 minutes):
```bash
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
```
```bash
. ~/.profile
```
```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest:$HOME/.local/bin' >> ~/.bashrc && source ~/.bashrc
```
First run (this compiles ArduCopter):
```bash
cd ~ && sim_vehicle.py -v ArduCopter --console --map
```
**Test:** wait for `EKF3 IMU0 is using GPS` and `pre-arm good`. Click in the terminal, press Enter until you see the `STABILIZE>` prompt, then type:
```
mode guided
```
```
arm throttle
```
```
takeoff 10
```
Type `takeoff 10` within 10 s of arming. ✅ Verified.

**Problem we hit:** typing the commands too early, before the prompt was ready, did nothing (the mode stayed STABILIZE and ARM stayed red). Wait for the prompt.

Manual flying, for practice:
- **Map method:** right-click on the map → **Fly to**.
- **Stick method:** in LOITER mode, use `rc 3 1500` (hold altitude), `rc 2 1400` (forward), `rc 2 1500` (stop).
- **Return home:** `mode rtl`.

## 6. `ardupilot_gazebo` plugin (course guide Step 5)
```bash
sudo apt install libgz-sim8-dev rapidjson-dev -y
```
```bash
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
```bash
export GZ_VERSION=harmonic
```
```bash
cd ~ && git clone https://github.com/ArduPilot/ardupilot_gazebo
```
```bash
cd ~/ardupilot_gazebo && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
```
```bash
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
```
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
```
```bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc && source ~/.bashrc
```
**Check:** the build ends with `[100%] Built target ArduPilotPlugin`.

## 7. ⚠️ The "frame class" bug in current ArduPilot `master` (important)
**Symptom:** with Gazebo, the drone never arms, and this repeats:
```
PreArm: Motors: Check frame class and type
```
**Cause:** current ArduPilot `master` (seen on 2 Nov 2026) no longer passes `--defaults` from `sim_vehicle.py`. Instead, the SITL binary looks up its embedded default parameters (`@ROMFS/vehicleinfo.json`) by the **`--model`** name. With Gazebo, the model is **`JSON`**, which has no entry, so **no default parameters load at all**. `FRAME_CLASS` stays 0, the motors can't initialise, and arming is refused. The `gazebo-iris` frame's own file (`Tools/autotest/default_params/gazebo-iris.parm`, which sets `FRAME_CLASS 1` and `FRAME_TYPE 1`) is never applied. The relevant code is `resolve_defaults_from_romfs()` in `libraries/AP_HAL_SITL/SITL_cmdline.cpp`, and the comment in `Tools/autotest/sim_vehicle.py` that says *"we no longer pass --defaults here"*.

**Fix:** pass the parameter files explicitly. Add `-w` the first time, to wipe any bad saved parameters:
```bash
cd ~ && sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map -w --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/copter.parm --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm --out=udp:127.0.0.1:14560
```
After that, use the same command without `-w`. `scripts/run_all.sh` does this for you.

**Manual alternative:** in the MAVProxy prompt, run `param set FRAME_CLASS 1`, `param set FRAME_TYPE 1`, then `reboot`.

The plain (non-Gazebo) SITL from Step 5 is not affected, because its model `quad` does have embedded defaults.

## 8. QGroundControl (course guide Step 6, optional)
```bash
sudo apt install libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
```bash
cd ~/Downloads && wget https://github.com/mavlink/qgroundcontrol/releases/download/v5.0.8/QGroundControl-x86_64.AppImage && chmod +x QGroundControl-x86_64.AppImage
```
Run it as a normal user, never with `sudo`. QGC listens on UDP 14550, which SITL feeds by default.

## 9. This project
```bash
cd ~ && git clone -b claude/wizardly-bohr-mxzqfb https://github.com/gusfring3369/Drone.git
```
```bash
sudo apt install python3-matplotlib -y
```
```bash
cd ~/Drone && python3 tests/test_mission.py
```
The test should print 3× PASS. If `pymavlink` is missing, install both packages:
```bash
python3 -m pip install --user pymavlink matplotlib
```
Then run the simulation as described in [`../README.md`](../README.md), or start everything with:
```bash
cd ~/Drone && ./scripts/run_all.sh long
```

---

## Quick troubleshooting index
| Symptom | Section |
|---|---|
| `apt`: no installation candidate / unable to locate git, curl | §2 |
| No internet in the VM / no network adapter | §2.1 |
| `glxinfo` shows `llvmpipe` | §2.4 |
| `Unable to locate package gz-harmonic` | §4 |
| Gazebo window flickers | §4 (`--render-engine ogre`) |
| `iris_runway.sdf` / `iris_with_ardupilot` not found | §6 (`GZ_SIM_RESOURCE_PATH`) |
| `PreArm: Motors: Check frame class and type` | §7 |
| Commands in MAVProxy do nothing | §5 (wait for the prompt) |
| `No heartbeat from the autopilot` (`mission.py`) | SITL not started with `--out=udp:127.0.0.1:14560` |
| `bcdedit` "The set command specified is not valid" in PowerShell | Quote the id: `'{current}'` (only relevant to dual boot) |
