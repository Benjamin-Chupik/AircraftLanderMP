# AircraftLanderMP
ASEN5254 final project. Creates a motion plan for a fixed wing aircraft to land on a runway.

How to run code:
Each goals code is seperated and stored in its respective folder starting with Goal_...
To run the goals code:
    1) enter in the folder and run the cmake file. This will generate a Build folder in that directory or whatever directory you specify.
    2) enter in the build folder -> debug which will have the planePlan.exe file
    3) run the planePlane.exe file which will generate the 2 output files for geometric and control data
    4) to plot the date return to the base folder for that goal (Goal_...) and run the python program plot.py
        This will generate the plots from the data files just generated in the debug folder. 
        Also the isSave variable can be changed to true or false to save the plots to the same directory

Note:
    To change the start conditions or goal conditions go into Goal_.../main.cpp
    In the plan with simple setup function, you can change the start position via the start variable. 
    In Goal 1 you can change the goal position using the goal variable in the simple setup function.
    For all other goals, change the goal cost function in the custom goal class.
    You can change the runtime for the program by changeing the input to ss.solve() in the planWithSimpleSetup function. (input in seconds)