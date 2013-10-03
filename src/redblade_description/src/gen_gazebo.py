import argparse
import xml.etree.ElementTree as ET
import os
import sys

parser = argparse.ArgumentParser(description=\
                                     'Parse URDF file to generate a starter gazebo file')
parser.add_argument(\
    '--urdf-file',type=str,required=True,
    help='The path of the URDF file')
parser.add_argument(\
   '--gazebo-file',type=str,required=True,
    help='The path of the URDF file')
args = parser.parse_args()

def handleLinks(links):
    pass

Gcolors = {"black":"Gazebo/Black",
           "grey":"Gazebo/Grey",
           "wheel":"Gazebo/Black",
           "white":"Gazebo/White",
           "green":"Gazebo/Green",
           "blue":"Gazebo/Blue",
           "gold":"Gazebo/Gold",
           "red":"Gazebo/Red",
           "purple":"Gazebo/Purple"}

if __name__=="__main__":
    out = open(args.gazebo_file,'w')
    tree = ET.parse(args.urdf_file)
    root = tree.getroot()
    print>>out,"<?xml version=\"1.0\"?>"
    print>>out, "<robot>"
    for link in root.findall('link'):
        for material in link.iter('material'):
            print>>out, "\t<gazebo reference=\"%s\">\n\t\t<material>%s</material>\n\t</gazebo>"%(link.attrib['name'], 
                                                                                                 Gcolors[ material.attrib['name'] ])
    print>>out, "</robot>"
