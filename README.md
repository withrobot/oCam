#oCam - 5MP USB 3.0 Color Board Camera
![ScreenShot](images/oCam_model.jpg)

* **Easy**: oCam does note require any device driver to be installed on the host PC or the Odroid-XU4
* **Versatile**: oCam supports the M12 lens format of various focal lengths for different needs
* **Low CPU usage**: oCam supports USB 3.0 with direct memory access, allowing data to be written to main memory without going through the CPU
* **Good for embedded system**: oCam is verified with the Odroid-XU4 embedded board from HardKernel™

##Board Detail
![ScreenShot](images/oCam_layout.png)

* 수치가 기입된 외형으로 변경할 예정

##Specifications
Pin | Description | Type | Description
------|------|------|-------------
**Sensor** | INT | O | DATA READY interrupt output
**Interface** | SLEEP | I | Sleep mode selection input: L-sleep mode, H-normal mode. (Normal mode If not connected)
**Lens** | I2C_SCL | I | I2C clock input 
**Supported OS** | I2C_SDA |  I/O  | I2C data input and output 
**Power** | USB_DM | I/O | USB D-
**Operation Temperature** | USB_DP | I/O | USB D+
**Shutter** | NC |   | Do Not Connect
**Camera Control** | NC |   | Do Not Connect
**Frame Rate** | NC |   | Do Not Connect
**Size** | NC |   | Do Not Connect
