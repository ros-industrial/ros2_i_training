## Introduction

The workshop documents are written in `Markdown` language and built using [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?) documentation generator.

## Usage
### Install dependencies

````bash
cd ~/ros2_i_training/workshop/
python3 -m pip install -r requirements.txt
````

### To add new content
Place the content source in the following folders in the appropriate session.
-  `_source`: Markup (.md) files 
- `_static`: Images and other resource files  

Add the workshop's heading and filepath relative to `~/ros2_i_training/workshop/source/_source` to `index.rst`. To build the html:
 ````bash
 cd ~/ros2_i_training/workshop/
 make html
 ````
`index.rst` is built into `index.html` in the documentation output directory `~/ros2_i_training/workshop/build/html/index.html`. 

![docs](/workshop/source/_static/demo_rtd.png)
