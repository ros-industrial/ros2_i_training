# Workshop
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
cd workshop
````
1.  Edit your `index.rst` and add some information about your project. Include as much detail as you like 
2.  Build them to see how they look:
 ````bash
 make html
 ````
3.  Your `index.rst` has been built into `index.html` in your documentation output directory ` _build/html/index.html`
4.  Open this file in your web browser to see your docs.
![docs](/workshop/source/_static/demo_rtd.png)

## Reference
- Hosting is done here [Dashboard](https://readthedocs.org/dashboard/)
- For syntax of the `index.rst` file refer to this [link](https://thomas-cokelaer.info/tutorials/sphinx/rest_syntax.html)
- Setting up [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?)

-----------------------

# Slides
## File structure
- Slides are structured as follows
    - static - location for all the images and videos of associated section
    - source - location for the `html` files

## Markdown preview enhanced with VSCode and Puppeteer:

* Install Chrome (or Chromium): 
  ```
  sudo apt install chromium-browser
  ```
* Install the extension Markdown Preview Enhanced in VSCode
* Configure the (File/Preference/Settings) by providing the path to Chromium (in a terminal which chromium-browser)


## Exporter:

* Install mume and decktape:
  ```
  sudo npm install -g decktape
  sudo npm install -g @shd101wyy/mume
  ```

 <!-- TODO: Automaticaly find path -->
 
* Set Chromium and npm bin path in the export.py script
  ```
  npm bin
  which chromium-browser
  ``` 

* Run the export.py script. It will export all pdf in the 'export' folder with one folder per day and sorted by index. 
    ````
    ptyhon3 export.py
    ````
-------------------------
# Appendix

**Here is all you need to know about relative file paths:**

 - **Starting with** `/` returns to the root directory and starts there
   
 - **Starting with** `../` moves one directory backward and starts there
   
 - **Starting with** `../../` moves two directories backward and starts there (and so on...)
   
 - **To move forward**, just start with the first sub directory and keep moving forward.
 
 -------------------------
