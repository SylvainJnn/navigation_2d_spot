#! /usr/bin/env python
import rosplan_pytools
import rosplan_pytools.rosplan.controller.knowledge_base as kb
import rosplan_pytools.rosplan.controller.scene_database as sdb
import rosplan_pytools.rosplan.controller.planning_system as ps
  
rosplan_pytools.init()
  
# Using the KB
#kb.add_instance("loc1", "location")
  
# You can store stuff into the scene database with a third arg
#kb.add_instance("msg1", "msg_type")
#sdb.add_element("msg1", sdb.Element(std_msgs.msg.String("Be sure to drink your ovaltine"), "msg_type"))

#kb.add_goal("robot-at", loc="loc1")
#kb.add_goal("has-received-message", msg="msg1", loc="loc1")
  
# Then, plan and execute! using PS
ps.plan()
  
# Now, let's try stopping it
time.sleep(2)
ps.cancel()
