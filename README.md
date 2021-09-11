# LegoControlDriver
Lego Control II to Arduino interface code and info.

This is the code related to the video here: https://www.youtube.com/watch?v=ZT2Cm_PV4GQ

There are plenty of comments in the source code, here's a rundown of the projects:

## Lego-Control-II-Driver 
This is the main Arduino Project - i.e. file - for the driver. Single file only so far, but uses the TimerInterrupt library.

## KiCAD
A simple schematic and PCB for interfacing the Arduino pinout with the 20 pin flat cable connector.

## Atkelar.LegoControlII
My C# library to communicate with the Arduino code. Written in .net core 3.1

## Atkelar.LegoPlotter
A C# command line application that wraps the plotter model and can print a simple HPGL file.

