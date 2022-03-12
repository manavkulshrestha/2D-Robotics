# roger #

The repository for socket version of Roger-the-Crab simulator.


### Setup Guide

SYSTEM REQUIREMENTS

1- You may need certain packages before installation. The neccessary packages on
   a fresh installation of Ubuntu 18.04 can be obtained by running:
```bash
sudo apt install make gcc libx11-dev libxt-dev libxmu-dev libxaw7-dev libgsl-dev
```

STEPS FOR COMPILING THE SIMULATOR

1- Unzip roger.zip

2- Go to RogerSimulator. run
```bash
make clean; make
```

3- Go to RogerClient run
```bash
make clean; make
```

4- Create a symbolic link between the generated libraries to RogerProjects and RogerSimulator. In each Directory you can run:
```bash
       ln -s ../RogerClient/lib/
```

4.1 - You may also need to create additional symbolic links for the include folder. In each Directory you can run:
```bash
       ln -s ../RogerClient/include/
```

The above steps need to be done only once.

5- Compile RogerProjects by running
```bash
make clean; make
```

This will generate an executable named roger

STEPS FOR RUNNING THE SIMULATOR

1- Open two terminal windows.
2- In the first window, change the current directory to RogerSimulator. In the other two windows, change the directory to RogerProjects.
3- In the first window run:
```bash
./simulator EnvironmentNum RobotNum
```
  ,where EnvironmentNum and RobotNum are integer arguments that determine the
  simulation environment and number of robots respectively.
  EnvironmentNum = 0 : ARENA
  EnvironmentNum = 1 : DEVELOPMENT
    This starts up the simulator. However, the simulator will not display until it has connection from RobotNum different rogers.
4- In the second window run:
```bash
./roger 127.0.0.1 8000
```
5- Repeat step 4 in other terminal windows with different port numbers RobotNum times:
```bash
  ./roger 127.0.0.1 8001
```

The port numbers start from 8000 for the first player and increment by 1 for each new
player. This will start the simulator with RobotNum Rogers.
