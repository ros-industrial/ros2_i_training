## Introduction

The slides are written in `Markdown` language and converter to slides using [remark](https://github.com/gnab/remark) tool. Please see the [documentation](https://github.com/gnab/remark/wiki) for further details.

## File structure
Slides are structured as follows for each session:  
`source`: html files  
`static`: Images, videos and other resource files 

## Usage
### Install dependencies

Install Chrome (or Chromium): 
  ```
  sudo apt install chromium-browser
  ```
The script `export.py` exports `html` to `pdf` files, which depends on `mume` and `decktape`
  ```
  cd ~/ros2_i_training/
  npm install decktape
  npm install @shd101wyy/mume
  ```
Modify Chromium and npm bin path in the `export.py` script, if necessary
  ```
  npm bin
  which chromium-browser
  ``` 

### Generate slides
Run `export.py` script. It will export all html files to `export` folder in PDF format.
```
python3 export.py
```
