1.) Turn the drone on (by inserting the battery plugs into each other)
2.) You have to enable your Mobile Hotspot on your base-station. We’re assuming you’re using your laptop as your base-station.
For Windows: Click the Windows Icon on your keyboard -> Enter “Mobile hotspot settings” -> Under Properties -> Click Edit -> Rename the Network name to “duckietown” and Network password to “quackquack”
Otherwise, use phone hotspot.
3.) Wait anywhere from a couple seconds to a couple minutes, a connected device called “drone2” or “drone1” (depends which drone you’re communicating with) will connect
The drone will communicate with your laptop through this network. Do not worry about latency, inputs from your keyboard are instantaneous actions on the drone (unless you fly out of your hotspot network range)
4.) Run: ssh duckie@drone2.local or ssh duckie@drone1.local (whatever connected to your hotspot) in your terminal
	password: quackquack
5.) Run these commands in your ssh
	cd ~/catkin_ws/src/pidrone_pkg
    rake start
    screen -c pi.screenrc
    [press enter on FC screen to start FC node. Get to FC by `1. Should have python flight_controller_node.py]
6.) Open /pidrone_pkg/web/index.html from the directory you cloned the pidrone_pkg github repo
7.) Use duckie@drone2.local  (drone 2) or duckie@drone1.local (drone 1) as the hostname in your web page
8.) you’re a good little boy for following the directions, you're ready to fly ;)