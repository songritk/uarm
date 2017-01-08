# uarm
Simple UARM Controll code

* `uarm-joy.py` ต้นฉบับสำหรับเซอร์โว UARM ที่มีสาย feedback
* `uarm-joy2.py` สำหรับเซอร์โวทั่วไป มีสาย 3เส้น
* `uarm-pixy.py` สำหรับควบคุมผ่าน pixy 

ไฟล์ทั้ง 3 รันบน RPi3(Raspberry PI3) มี UARM และ Pixy ต่อ RPi3 

โค้ด `uarm-joy2.py` และ `uarm-pixy.py` ใช้แขนกล UARM แต่ใช้เซอร์โวทั่วไป (WK-M1500) และ BASED ใช้ SM-S4315R

ติดตั้ง library สำหรับ uarm และ pixy ก่อนรันโปรแกรม
# uarm-pixy.py
1. กำหนด object ที่สนใจบนคอมพิเตอร์ ผ่านโปรแกรม pixymon 
2. คอมไพล์ pantilt_in_python แล้ว copy ไว้ไดเร็คทอรี่เดียวกับ uarm-pixy.py 
2. ต่อสาย USB-UARM กับ RPi3 , ต่อสาย Pixy กับ RPi3
3. รันโปรแกรม `./uarm-pixy.py`
