#!/usr/bin/env python3
import os
import yaml
import subprocess
import shutil
import shlex
from pathlib import Path

chromepath="/usr/bin/chromium-browser"
home= str(Path.home())
root_dir = os.path.dirname(os.path.abspath(__file__))
export_dir = os.path.join(root_dir, 'export')
npm_bin = os.path.join(root_dir, 'node_modules/.bin')

if os.path.exists(export_dir):
    print("Clearing export dir")
    shutil.rmtree(export_dir)
    os.makedirs(export_dir)
else:
    print("Exporting pdfs to {}".format(export_dir))
    os.makedirs(export_dir)
deck_tape_cmd = '%s/decktape remark --allow-file-access-from-files true --chrome-arg=--disable-web-security'%(npm_bin)
node_cmd = 'node common/workshop_export.js %s'%chromepath


def export_slides(day_dir, id, slides):
    slides_path = os.path.join(root_dir, slides)
    name = os.path.splitext(os.path.basename(slides_path))[0]
    out_name = "%s_%s.pdf"%(id, name)
    out_path = os.path.join(day_dir, out_name)
    cmd = shlex.split(deck_tape_cmd)
    cmd.append(slides_path)
    cmd.append(out_path)
    subprocess.run(cmd)
    print("Created Slides: %s"%(out_path))

def export_workshops(day_dir, id, workshop):
    ws_path = os.path.join(root_dir, workshop)
    name = os.path.splitext(os.path.basename(workshop))[0]
    name_pdf = "%s.pdf"%name
    pdf_file = os.path.join(os.path.dirname(ws_path),name_pdf)
    out_name = "%s_WS_%s"%(id, name_pdf)
    out_path = os.path.join(day_dir, out_name)
    cmd = shlex.split(node_cmd)
    cmd.append(ws_path)
    subprocess.run(cmd)
    shutil.move(pdf_file, out_path)
    print("Created Workshop: %s"%(out_path))


def export_pdf(config_yaml):
    with open(config_yaml,'r') as config_file:
        config = yaml.safe_load(config_file)
    for day in config['export']:
        day_id = "Day%s"%day['Day']
        day_dir = os.path.join(export_dir, day_id)
        os.makedirs(day_dir)
        for id, slides in day['slides'].items():
            export_slides(day_dir, id, slides)

export_pdf(os.path.join(root_dir, "export_list.yaml"))

