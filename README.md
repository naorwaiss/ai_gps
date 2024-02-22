# AI- GPS 
this project come to gave solution to the GPS with AI 



at this project i use asus tinker board with dorne 
at the drone i use ardupilot - for the telemetry esc 
--------------------------------------------------------------------------------
missions that not do
1) build the drone X2 with 2 type of engine 
2) make the ESC telemtry
3) learn pytorch - control it 
4) start make the image code with the euiler or quarterion angel 
5) learn how to get the ESC data to pymavlink or mavsdk at real time
6) make code that shave all the data to csv file 
7) make the nn
8) check the netwaork 


--------------------------------------------------------------------------------
download mavlink 
# need to download and check if the tinker board 2s get pip - if not, do this setup

1) wget https://bootstrap.pypa.io/get-pip.py
2) sudo python3 get-pip.py
3) pip3 --version --- check the version and if the pip install
if it work - do install with the virtual enveirment 
4) python3 -m venv myenv
5) source myenv/bin/activate
6) pip install --upgrade pip
7) deactivate


connect to the tinker board to the MavLink:
dont tink that i need to move do "chmod"
sudo mavproxy.py --master=/dev/ttyS0


---------------------------------------------------------------------------------




