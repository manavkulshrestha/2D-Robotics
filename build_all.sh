#!/bin/bash
cd RogerClient;
make clean;
make;
cd ../RogerSimulator;
make clean;
make;
cd ../RogerProjects;
make clean;
make;
