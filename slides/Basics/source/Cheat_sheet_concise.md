# Cheat sheet

###### ROS 2 package installation syntax:   
`sudo apt install ros-foxy-<pkg_name>`

###### Basic ROS 2 command structure:   
`ros2 <main_command> <sub_command> [--options] <list_of_arguments>`    

Remember, you can use double `[tab][tab]` completion to find list of available options and `-h` to get some help text at any point.

Every ROS 2 command and sub-command has a list of options that you can use to modify its behavior. The list of options can be seen with `-h` and included as desired.   

You can use `| grep <string>` to filter results using a string.   

## Commonly used commands
Sourcing ROS 2:   
* `source /opt/ros/foxy/setup.bash`

Alias:
>WARNING: Use with caution!
* `echo "alias source_foxy=source /opt/ros/foxy/setup.bash" >> .bashrc`  

Packages introspection:    
* `ros2 pkg list` - list all available packages   
* `ros2 pkg executables <<optional_pkg_name>>` - list all available executables     

Node execution:   
* `ros2 run <package_name> <executable_name> <<optional_command_line_arguments>>` - start a node from a package  

Node introspection:   
* `ros2 node list` - list all running nodes   
* `ros2 node info <node_name>` - show information about a node     

Topic introspection:   
* `ros2 topic list -t` - list all discoverable topics with type info      
* `ros2 topic info <topic_name>` - get info about a topic   
* `ros2 topic echo <topic_name>` - echo the messages receved on a topic   
* `ros2 topic pub <topic_nam> <msg_type> "{<field1_key>: <field1_value>,  <field2_key>: <field2_value>, ...}"` - publish a message to a topic. fieldx_key and fieldx_value form a dictionary for the content of the message. Precise names for keys follow from message type      

Message introspection:   
* `ros2 interface list -m` - list all available message types   
* `ros2 inerface show <msg_name>` - show a specific message type (dictionary key names)  
* `ros2 interface proto <msg_name>` - show the prototype for using this msg in the terminal

Service introspection:   
 * `ros2 service list` - list all available services   
 * `ros2 service call <service_name> <service_type_name> "{<field1_key>: <field1_value>,  <field2_key>: <field2_value>, ...}"`- call a service. fieldx_key and fieldx_value form a dictionary for the content of the srv. Precise names for keys follow from srv type

 Service type introspection:   
 * `ros2 interface list -s` - list all available service types   
 * `ros2 interface show <msg_name>` - show a specific service type (dictionary key names)  

 Action introspection:   
 * `ros2 action list -t` - list all available actions, with their types
 * `ros2 action info <action>`  - get more info on an action
 * `ros2 action send_goal <goal>`   - send a goal to an action
 
 Action type intospection:   
 * `ros2 interface show <type>`  - show more info on the action type  

 Parameters:
 * `ros2 param list` - list names of all parameters available
 * `ros2 param describe <node> <param>` - describe datatype info of the param 
 * `ros2 param get <node> <param>` - get value of a param in a node
 * `ros2 param set <node> <param> <value>` - set a new value for the param
 * `ros2 param dump <node_name>` - write all current param values of a node into a file. This can later be loaded by a node to re-obtain the saved values. 

RQT:   
`rqt` - opens the rqt pane, loading in the last used plugin(s)   
Commonly used plugins: `Introspection->Node graph`, `Topics->Message publisher`, `Topics->Topic monitor`, `Services>Service caller`.   

Ros2bags:   
* `ros2 bag record [topics list]` - record to a ros2bag.   Useful options:   
    * `-a` - record all topics
    * `-o` - define a specific name for the db3 file, defaults to timestamp instead
* `ros2 bag info <ros2bag name>` - show info about the recorded ros2bag
* `ros2 bag play <ros2bag name>` - play the selected bag. 