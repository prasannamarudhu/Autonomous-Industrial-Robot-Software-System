# Industrial-Robot-Software-System


Contents:
   - config
   - include
   - launch
   - src
   - CMakeLists.txt
   - package.xml
   - pick_place.sh

------HOW TO RUN:  -----------------
Follow the instructions :

Extract the package in your workspace and build it using: 

1. catkin_make 
2. source devel/setup.bash

Give permissions to the script named "pick_place.sh" by making your present working directory as the package (need xterm installed to run this, install using "sudo apt-get install xterm") and do: 
	
3. cd /path/to/package/
4. sudo chmod a+x pick_place.sh

Run the packages by running the script: 

5 ./pick_place.sh


OUTPUT VIDEO:
https://drive.google.com/file/d/1vlRRMg-TqGWne7PGr1cxZcfWKAH3tVVW/view