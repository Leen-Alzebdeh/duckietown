# How to use the code:

1.  Clone the folder to your machine. 

2.  Before you run the code, please navigate to `/packages/move_robot/src/drive.py` and make changes to the radius of the wheels and the length between the wheels and centre of rotation if necessary. 

3.  After that open this folder in the terminal. Once you are in the folder, type in the terminal: `dts devel build -f -H [Vehicle_name]`
to build the program in the duckiebot. Replace `[Vehicle_name]` with your robot's name.

4.  To run the program, type in the terminal: `dts devel run -H [Vehicle_name].local`. Replace `[Vehicle_name]` with your robot's name.
