
---
# 3 DOF ROBOT

## Part 1: Setup Environment (2 คะแนน)
clone github ลงมาผ่านคำสั่ง
```bash
cd
git clone https://github.com/Phetzxc/fun4_38.git
cd fun4_38
colcon build
source install/setup.bash
echo 'source ~/fun4_38/install/setup.bash' >> ~/.bashrc
```
### 1. หา workspace ของแขนกลพร้อมวิธีการตรวจสอบคําตอบ (1 คะแนน)
แสดง workspace ของแขนกล ใช้คำสั่งดังนี้:

```bash
ros2 launch example_description launch_all.launch.py
```
ภาพจำลองworkspaceจะขึ้นมาทุกครั้ง

![image](https://github.com/user-attachments/assets/6b8da4d4-0b3d-458e-bb2e-db1bf647973f)
### 2. Node สำหรับสุ่มเป้าหมาย (0.5 คะแนน)
สร้าง node ที่สุ่มตำแหน่งเป้าหมายของแขนกลภายใน workspace และ pub topic `/target`:

เพื่อดูตำแหน่งของเป้าหมายใน **RVIZ2** ให้ใช้คำสั่ง:

```bash
ros2 topic echo /target
```
![image (1)](https://github.com/user-attachments/assets/3cc38cca-16ac-43f8-840e-7066ffc44e11)

### 3. Node สำหรับส่งค่าตําแหน่งปลายมือ (0.5 คะแนน)
ส่งตำแหน่งปลายมือของแขนกลไปยัง topic `/end_effector`:

```bash
ros2 topic echo /end_effector
```
![endeff](https://github.com/user-attachments/assets/d8e3dab6-41f2-40d2-be1c-e1b3fd2b50f6)

ทั้งตำแหน่งของ `/target` และ `/end_effector` จะถูกแสดงใน **RVIZ2** โดยตำแหน่งเริ่มต้นของ `/target` จะอยู่ที่ `(0, 0, 0)` และจะถูกสุ่มใหม่ในโหมด Auto

![endeff2](https://github.com/user-attachments/assets/9101e772-d7e4-4bae-9fe9-7da73762445e)

---

## ส่วนที่ 2: Controller (7 คะแนน)

แขนกลสามารถทำงานได้ใน 3 โหมด:

- **Inverse Pose Kinematics (IPK)** 
- **Teleoperation (Teleop)** 
- **Autonomous (Auto)**

### 1. เปิดใช้งาน Node

```bash
ros2 launch example_description launch_all.launch.py
```

จากนั้นเปิด Node สำหรับควบคุมโหมดต่าง ๆ:

```bash
ros2 run example_description controller.py
```
![git](https://github.com/user-attachments/assets/7df18872-1e62-4eba-9753-c40944914292)
### 2. Control mode

#### mode 0: Inverse Pose Kinematics (IPK) – (2 คะแนน)
ในโหมดนี้ระบบจะคำนวณตำแหน่งของข้อต่อ (joint) เพื่อให้ไปถึงตำแหน่งที่กำหนดใน task-space

- ตัวอย่างคำสั่ง:
  ```bash
  ros2 run example_description controller.py 0 <X> <Y> <Z>
  ```
![git2](https://github.com/user-attachments/assets/fcfe508a-35b7-4635-8e86-bf7931b655d0)

- หากตำแหน่งนั้นอยู่ภายใน workspace ระบบจะส่งคำสั่งให้แขนกลเคลื่อนที่ไปยังเป้าหมาย พร้อมส่งค่า solution กลับมา
- หากตำแหน่งนั้นอยู่นอก workspace ระบบจะส่งค่า `False` กลับมา และแขนกลจะไม่เคลื่อนที่ไปไหน

#### mode 1: Teleoperation – (2 คะแนน)
ในโหมดนี้ผู้ใช้สามารถควบคุมแขนกลด้วยมือผ่าน topic `/cmd_vel` (Twist message) โดยมีการควบคุม 2 แบบ:

- **Reference จากปลายมือ**
- **Reference จากฐานของแขนกล**

- ตัวอย่างคำสั่ง:
  ```bash
  ros2 run example_description controller.py 1 <0 หรือ 1>
  ```
![git3](https://github.com/user-attachments/assets/46e712d6-c139-48d5-9470-23ab283faca3)

 `0` หมายถึง Reference กับปลายมือ และ `1` หมายถึงReferenceจากฐานของหุ่นยนต์

หลัง run จะมีหน้าต่างเด้งขึ้นมาเพื่อให้ใช้ควบคุมทิศทางด้วยปุ่ม:
- `a`, `s`, `d`: เคลื่อนที่แกน x, y, z
- `f`, `g`, `h`: เคลื่อนที่ในทิศทางตรงกันข้าม
- `q`: ออกจากโหมด Teleoperation
- 
![git5](https://github.com/user-attachments/assets/00b86bc0-dde3-4ff2-955c-e27f698d02d3)

หุ่นยนต์จะหยุดเคลื่อนที่ทันทีหากตรวจพบว่าสภาพการเคลื่อนที่เข้าสู่ singularity และจะมีการแจ้งเตือนผ่านข้อความ

![git7](https://github.com/user-attachments/assets/99a100cd-e365-4459-9447-0fc97dfe6863)

#### โหมด 2: Auto – (2 คะแนน)
ในโหมดนี้แขนกลจะเคลื่อนที่ไปยังตำแหน่งที่สุ่มได้ภายใน workspace โดยมีเวลา 10 วินาทีเพื่อไปถึงเป้าหมาย

- คำสั่ง:
  ```bash
  ros2 run example_description controller.py 2
  ```
![git4](https://github.com/user-attachments/assets/62fed749-d1d6-4176-bee8-9c7eb9a20dc6)

หากไปถึงเป้าหมายสำเร็จ ระบบจะส่งข้อความกลับมา และเริ่มการสุ่มเป้าหมายใหม่ต่อไป

![git8](https://github.com/user-attachments/assets/4eda572d-afac-4a29-8ddb-8e8410f6cf74)

---

## change mode (1 คะแนน)

การเปลี่ยนโหมดสามารถทำได้ด้วยคำสั่ง:

- **Mode 0**: IPK Mode
- **Mode 1**: Teleoperation Mode
- **Mode 2**: Autonomous Mode

เมื่อการเปลี่ยนโหมดสำเร็จ ระบบจะส่งค่า `True` กลับมา

---

## ตัวอย่างคำสั่ง

- **โหมด IPK (Inverse Pose Kinematics)**:
  ```bash
  ros2 run example_description controller.py 0 0.2 0.1 0.3
  ```

- **โหมด Teleoperation**:
  ```bash
  ros2 run example_description controller.py 1 1
  ```

- **โหมด Autonomous**:
  ```bash
  ros2 run example_description controller.py 2
  ```

---
## ทดเขียน diagram และrqt_graph ที่ได้
- **กระดาษทด diagram**
![CLIENT1](https://github.com/user-attachments/assets/1dee51a8-9821-417a-abce-c6dc99c3015f)
- **rqt_graph**
![image](https://github.com/user-attachments/assets/10a14844-267c-47cf-b049-a067421c31d9)

---
