## Download spacemouse dependencies 

```bash
sudo apt install libspnav-dev spacenavd; sudo systemctl start spacenavd
pip install spnav
```

## Check if spacemouse is connected to workstation
```bash
lsusb 
```

## Download RTDE library 
```bash
pip install -- user ur_rtde 
```

## Run spacemouse script 
```bash
python3 3DConnexion_UR5_Teleop.py
```

## Run spacemouse script with gripper position control 
```bash
python3 3DConnexion_UR5_Teleop_Gripper_Control.py
```

## Note:
In the spnav library, PyCObject_AsVoidPtr is deprecated 

find . -name "spnav" on terminal to find spnav folder 

Replace all instances of PyCObject_AsVoidPtr with PyCapsule_GetPointer in __init__.py 

## To include more RTDE functionalities
https://sdurobotics.gitlab.io/ur_rtde/index.html
