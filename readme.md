[![Documentation Status](https://readthedocs.org/projects/ros2-workshop/badge/?version=latest)](https://ros2-workshop.readthedocs.io/en/latest/?badge=latest)

[ROS2-foxy-workshop](https://ros2-workshop.readthedocs.io/)
## To add new content
- Markup(.md) files should reside in the `_source` folder
- Images should be inside `_static`
- Always add the filepath to the `index.rst` file.

## Usage
### Install dependencies
````bash
python3 -m pip install -r requirements.txt
````
### Getting Started with Sphinx
````bash
cd docs
sphinx-quickstart
````
1. This quick start will walk you through creating the basic configuration; in most cases, you can just accept the defaults. When it’s done, you’ll have an `index.rst`, a `conf.py` and some other files. 

2.  Now, edit your `index.rst` and add some information about your project. Include as much detail as you like 
3.  Build them to see how they look:
 ````bash
 make html
 ````
4.  Your `index.rst` has been built into `index.html` in your documentation output directory ` _build/html/index.html`
5.  Open this file in your web browser to see your docs.
![docs](/demo.png)

## Reference
- For syntax of the `index.rst` file refer to this [link](https://thomas-cokelaer.info/tutorials/sphinx/rest_syntax.html)
- Setting up [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?)