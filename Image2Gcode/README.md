# Convert your images to Gcode

## 📌 Description
This project allows you to convert images into G-code files for use with CNC machines, pen plotters, or laser engravers. The program also creates a report in Excel format with processing logs and thumbnails.

---

## ⚙️ Requirements 
## ⬇️ Install Python 3.9

### 🔹 Windows

1. Download Python 3.9 from : https://www.python.org/ftp/python/3.9.0/python-3.9.0-amd64.exe  
2. Confirm installation:
```bash
python --version
```
# 🔧 Create and Set Up Virtual Environment (Python 3.9)
1. Create a virtual environment:
```bash
$ python3.9 -m venv .venv
```
2. Activate the virtual environment:
```bash
$ ./.venv/Scripts/Activate  #  Windows
```
3. Upgrade pip inside the virtual environment:
```bash
python.exe -m pip install --upgrade pip
```
4. Install required packages:
```bash
pip install -r requirements.txt
```
## 🚀 Usage

🖼️ Convert image to G-code
```bash
$ python genGcode.py
```
✅ Output will be created and saved in the /out directory

📊 An Excel report will be generated in : 
/excel/time_processing.xlsx

📤 Upload G-code to Arduino (Optional)
```bash
$ python GcodeSender.py
# make sure the PORT is the same as your environment (COM4 for example)
```


