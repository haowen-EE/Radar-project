# 📁 How to Manually Modify Data Files

## Method 1: Directly Modify Script Configuration (Recommended) ✨

Open `csv_boxs_withbike_V5.py`, find the configuration area at **lines 88-93**:

```python
# =====================================================
# 📁 Manual Data File Configuration Area
# =====================================================
MANUAL_CSV_FILE = 'escooter1.csv'

# Set to None to use command line arguments or auto-detection
# MANUAL_CSV_FILE = None
# =====================================================
```

### Example 1: Use File in Current Directory

```python
MANUAL_CSV_FILE = 'escooter2.csv'
```

### Example 2: Use File in Subfolder (Relative Path)

```python
MANUAL_CSV_FILE = 'DATA_V1/escooter_fast_with_RSC.csv'
```

### Example 3: Use File in Other Location (Absolute Path)

```python
MANUAL_CSV_FILE = r'C:\Users\h\Desktop\my_data.csv'
```

> ⚠️ **Note**:
> - Relative paths are relative to the script directory
> - Use `r` prefix for absolute paths or use `/` instead of `\`
> - If file doesn't exist, will directly report error

### Example 4: Disable Manual Configuration, Use Command Line Arguments

```python
MANUAL_CSV_FILE = None
```

---

## Method 2: Use Command Line Arguments (No Code Modification Needed)

```powershell
# Specify filename
python csv_boxs_withbike_V5.py --csv escooter2.csv

# Specify relative path
python csv_boxs_withbike_V5.py --csv DATA_V1/escooter_slow_with_RSC.csv

# Specify absolute path
python csv_boxs_withbike_V5.py --csv "C:\Users\h\Desktop\my_data.csv"
```

---

## Method 3: Create Data Folder (Auto-Detection)

1. Create `Data` folder in script directory
2. Copy your CSV file to `Data` folder
3. Ensure filename contains `scooter` keyword
4. Run script, it will automatically use first file found

```powershell
mkdir Data
copy my_radar_data_scooter.csv Data\
python csv_boxs_withbike_V5.py
```

---

## View Available Files

List all available scooter CSV files:

```powershell
python csv_boxs_withbike_V5.py --list
```

---

## CSV File Format Requirements

Your CSV file must contain the following columns:

| Column | Description | Required |
|--------|-------------|----------|
| `detIdx` | Detection index, 0 means new frame starts | ✅ Yes |
| `x` | X coordinate (meters) | ✅ Yes |
| `y` | Y coordinate (meters) | ✅ Yes |
| `z` | Z coordinate (meters) | ✅ Yes |
| `timeStamp` | Timestamp | ❌ No (optional)|

### Sample CSV Format:

```csv
detIdx,x,y,z,timeStamp
0,1.234,0.567,0.890,2025-01-15T10:30:45.123
1,1.456,0.678,0.901,2025-01-15T10:30:45.123
2,1.678,0.789,0.912,2025-01-15T10:30:45.123
0,1.890,0.890,0.923,2025-01-15T10:30:45.173
...
```

---

## Common Questions

### ❓ What if file not found?

Script will try the following paths (in order):
1. Your specified original path
2. `script_directory/your_filename`
3. `script_directory/Data/your_filename`

If all not found, will display warning and try auto-search.

### ❓ How to use Chinese paths?

```python
MANUAL_CSV_FILE = r'C:\用户\张三\桌面\数据.csv'
```

### ❓ Script runs but no response?

Check:
1. Is correct Python interpreter selected (Python 3.11.9 venv311)
2. Does CSV file exist and is format correct
3. Check console for error messages

---

## Recommended Configuration

For daily use, recommend setting at script beginning:

```python
# Development testing
MANUAL_CSV_FILE = 'escooter1.csv'

# Production environment
MANUAL_CSV_FILE = None  # Use command line arguments or auto-detection
```

---

**After modification, directly run script or click VS Code run button!** 🚀
