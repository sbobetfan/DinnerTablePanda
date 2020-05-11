#!/usr/bin/env python

import re
import random
import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import inquirer
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from collections import Counter




class HouseholdPandaArm(object):
  """HouseholdPandaArm"""
  def __init__(self):
    super(HouseholdPandaArm, self).__init__()

    ## BEGIN setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('PANDA_TABLE', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this program the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    print ""


    # Misc variables

    # Knife, fork, and plate objects all have same height and width
    self.obj_height = 0.15
    self.obj_width = 0.06

    self.z_coordinate_above_obj = 0.38

    # Table coordinates
    self.table_name = "table"
    self.table_x = 0.4
    self.table_y = 0
    self.table_z = 0.07


    # Table dimensions
    self.table_size_x = 0.4
    self.table_size_y = 0.8
    self.table_size_z = 2 * self.table_z

    # Table boundary coordinates
    self.table_lower_x = (self.table_x) - (self.table_size_x * 0.5)
    self.table_upper_x = (self.table_x) + (self.table_size_x * 0.5)
    self.table_lower_y = (self.table_y) - (self.table_size_y * 0.5)
    self.table_upper_y = (self.table_y) + (self.table_size_y * 0.5)

    #Knife, fork and plate objects will be same height, so have same z coordinates
    self.obj_z_coordinate = ((self.table_size_z) + ((self.obj_height)*0.5))

    # Plate coordinates
    self.plate_name = "plate"
    self.plate_x = 0.4
    self.plate_y = 0
    self.plate_z = self.obj_z_coordinate

    #Plate dimensions
    self.plate_size_x = 0.12
    self.plate_size_y = self.obj_width
    self.plate_size_z = self.obj_height


    # Knife coordinates
    self.knife_name = "knife"
    self.knife_x = self.plate_x
    self.knife_y = self.plate_y - 0.2
    self.knife_z = self.obj_z_coordinate

    # Knife dimensions
    self.knife_size_x = 0.08
    self.knife_size_y = self.obj_width
    self.knife_size_z = self.obj_height


    # Fork coordinates
    self.fork_name = "fork"
    self.fork_x = self.plate_x
    self.fork_y = self.plate_y + 0.2
    self.fork_z = self.obj_z_coordinate

    # Fork dimensions
    self.fork_size_x = 0.04
    self.fork_size_y = self.obj_width
    self.fork_size_z = self.obj_height


    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  global perform_move
  def perform_move(move_group, waypoints, obj_attached):
      fraction = 0.0
      maxtries = 10
      attempts = 0
      if obj_attached:
          sleeptime = 1
      else:
          sleeptime = 0.5
      # raise_attempted = False

      while fraction < 1.0 and attempts < maxtries:
          (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0, True)
          attempts += 1


          if attempts == maxtries:
              # print("Still trying after " + str(attempts) + " attempts...")

              break




      if fraction == 1.0:
          if obj_attached:
              print "Path computed successfully. Moving the arm WITH OBJECT."
          else:
              print "Path computed successfully. Moving the arm."

          move_group.execute(plan)
          time.sleep(sleeptime)
          print "Path execution complete."


          return True
      else:
          print("Path planning UNSUCCESSFUL : " + str(fraction) + " success after " + str(attempts) + " attempts.")
          return False

  global set_waypoints
  def set_waypoints(self, move_group, coordinates, obj_attached):
      waypoints = []

      #first move to object
      wpose = move_group.get_current_pose().pose

      if obj_attached:
          wpose.position.z += (self.obj_height + 0.05)
      else:
          wpose.position.z = (self.z_coordinate_above_obj + 0.05)

      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = coordinates[0]
      #waypoints.append(copy.deepcopy(wpose))

      wpose.position.y = coordinates[1]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.z = self.z_coordinate_above_obj
      waypoints.append(copy.deepcopy(wpose))
      return waypoints




  def calibrate_arm(self):
      move_group = self.move_group


      table_size_x = self.table_size_x
      table_size_y = self.table_size_y
      table_size_z = self.table_size_z

      waypoints = []

      wpose = move_group.get_current_pose().pose
      wpose.position.y -= (table_size_y / 2)
      wpose.position.x -= (table_size_x / 2)
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x += (table_size_x)
      waypoints.append(copy.deepcopy(wpose))



      wpose.position.y += table_size_y
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x -= (table_size_x)
      waypoints.append(copy.deepcopy(wpose))



      wpose.position.y -= (table_size_y / 2)
      waypoints.append(copy.deepcopy(wpose))


      wpose.position.x = self.table_x
      wpose.position.y = self.table_y
      waypoints.append(copy.deepcopy(wpose))

      move = perform_move(move_group, waypoints, False)





  def attach_object(self, obj_name):

    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names


    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, obj_name, touch_links=touch_links)

    return

  def detach_object(self, obj_name):

    scene = self.scene
    eef_link = self.eef_link

    move_group = self.move_group
    current_pos = move_group.get_current_pose().pose
    pos_x = float("%.2f" % (current_pos.position.x))
    pos_y = float("%.2f" % (current_pos.position.y))


    if obj_name == 'plate':
        self.plate_x = pos_x
        self.plate_y = pos_y
    elif obj_name == 'knife':
        self.knife_x = pos_x
        self.knife_y = pos_y
    elif obj_name == 'fork':
        self.fork_x = pos_x
        self.fork_y = pos_y


    scene.remove_attached_object(eef_link, name=obj_name)

    return




  def add_object(self, obj_name, obj_xyz, obj_dims):

    scene = self.scene


    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = "panda_link0"
    ##box_pose.pose.orientation.w = 1.0
    obj_pose.pose.position.x = obj_xyz[0]
    obj_pose.pose.position.y = obj_xyz[1]
    obj_pose.pose.position.z = obj_xyz[2]

    scene.add_box(obj_name, obj_pose, size=(obj_dims[0], obj_dims[1], obj_dims[2]))

    return

  def check_object(self, obj_name, rightful_coordinates, obj_notattached):
      if obj_name == 'plate':
          pos_x = self.plate_x
          pos_y = self.plate_y
      elif obj_name == 'knife':
          pos_x = self.knife_x
          pos_y = self.knife_y
      elif obj_name == 'fork':
          pos_x = self.fork_x
          pos_y = self.fork_y

      if (pos_x == rightful_coordinates[0]) and (pos_y == rightful_coordinates[1]):
          return True
      else:
          move_group = self.move_group
          coordinates = (pos_x, pos_y)

          waypoints = set_waypoints(self, move_group, coordinates, obj_notattached)

          move = perform_move(move_group, waypoints, obj_notattached)

          return False




  def getRandomFloat(self, type):
      if type == 'x':
          lower = self.table_lower_x + 0.1

          upper = self.table_upper_x - 0.1

      else:
          lower = self.table_lower_y + 0.05

          upper = self.table_upper_y - 0.05

      rand = random.uniform(lower, upper)

      return rand


  global intersection
  def intersection(a, b):
      a0, a1 = a
      b0, b1 = b

      return max(0, min(a1,b1)-max(a0, b0))

  global intersect_check
  def intersect_check(self, obj_id, rightful_coordinates):

      x,y = rightful_coordinates

      check1 = False
      check2 = False
      position_free = False

      p_x = self.plate_x
      p_y = self.plate_y
      p_depth = self.plate_size_x / 2
      k_x = self.knife_x
      k_y = self.knife_y
      k_depth = self.knife_size_x / 2
      f_x = self.fork_x
      f_y = self.fork_y
      f_depth = self.fork_size_x / 2
      obj_width = self.obj_width

      plate_x_boundaries = [(p_x - p_depth), (p_x + p_depth)]
      plate_y_boundaries = [(p_y - obj_width), (p_y + obj_width)]
      knife_x_boundaries = [(k_x - k_depth), (k_x + k_depth)]
      knife_y_boundaries = [(k_y - obj_width), (k_y + obj_width)]
      fork_x_boundaries = [(f_x - f_depth), (f_x + f_depth)]
      fork_y_boundaries = [(f_y - obj_width), (f_y + obj_width)]

      if obj_id == 'plate' :
          plate_x_boundaries = [(x - p_depth), (x + p_depth)]
          plate_y_boundaries = [(y - obj_width), (y + obj_width)]

          main_obj = [plate_x_boundaries, plate_y_boundaries]

          obj1 = [knife_x_boundaries, knife_y_boundaries]
          obj2 = [fork_x_boundaries, fork_y_boundaries]


      elif obj_id == 'knife' :
          knife_x_boundaries = [(x - k_depth), (x + k_depth)]
          knife_y_boundaries = [(y - obj_width), (y + obj_width)]

          main_obj = [knife_x_boundaries, knife_y_boundaries]

          obj1 = [plate_x_boundaries, plate_y_boundaries]
          obj2 = [fork_x_boundaries, fork_y_boundaries]


      elif obj_id == 'fork' :
          fork_x_boundaries = [(x - f_depth), (x + f_depth)]
          fork_y_boundaries = [(y - obj_width), (y + obj_width)]

          main_obj = [fork_x_boundaries, fork_y_boundaries]

          obj1 = [plate_x_boundaries, plate_y_boundaries]
          obj2 = [knife_x_boundaries, knife_y_boundaries]


      intersection_x1 = intersection(main_obj[0], obj1[0])
      intersection_y1 = intersection(main_obj[1], obj1[1])
      intersection_x2 = intersection(main_obj[0], obj2[0])
      intersection_y2 = intersection(main_obj[1], obj2[1])

      if intersection_x1 == 0:
          check1 = True

      elif intersection_y1 == 0:
          check1 = True

      if intersection_x2 == 0:
          check2 = True

      elif intersection_y2 == 0:
          check2 = True

      if check1 and check2:
          position_free = True


      return position_free


  def try_move_to_goal(self, obj_id, rightful_coordinates, obj_attached):
      check = intersect_check(self, obj_id, rightful_coordinates)

      move_group = self.move_group

      if check:
          waypoints = set_waypoints(self, move_group, rightful_coordinates, obj_attached)

          move = perform_move(move_group, waypoints, obj_attached)

      else:
          temp_place_found = False
          while not temp_place_found:

              print(">>>>>>>>>>>>>>> SPACE TAKEN, moving to a random position for now")
              temp_place_found = False
              new_x = random.uniform((self.table_lower_x + 0.12), (self.table_upper_x - 0.12))
              new_x_float = float("%.2f" % new_x)
              new_y = random.uniform((self.table_lower_y + 0.05), (self.table_upper_y - 0.05))
              new_y_float = float("%.2f" % new_y)
              new_coordinates = (new_x_float, new_y_float)

              check = intersect_check(self, obj_id, new_coordinates)
              if check:
                  waypointsB = set_waypoints(self, move_group, new_coordinates, obj_attached)
                  moveB = perform_move(move_group, waypointsB, obj_attached)
                  print("New Space Found")

                  temp_place_found = True
      return

  def set_coordinates(self, obj_name, coordinates):
      if obj_name == self.plate_name:
          self.plate_x = coordinates[0]
          self.plate_y = coordinates[1]
      elif obj_name == self.knife_name:
          self.knife_x = coordinates[0]
          self.knife_y = coordinates[1]
      elif obj_name == self.fork_name:
          self.fork_x = coordinates[0]
          self.fork_y = coordinates[1]

  def finish_move(self):
      move_group = self.move_group

      waypoints = []

      wpose = move_group.get_current_pose().pose

      wpose.position.z = self.z_coordinate_above_obj + 0.2
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = 0.4
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.y = 0
      waypoints.append(copy.deepcopy(wpose))



      move = perform_move(move_group, waypoints, False)



###############################################################################
###############################################################################
#############################  MAIN BELOW  ####################################
###############################################################################
###############################################################################

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the household Panda!"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the service ..."
    raw_input()
    DinnerTablePanda = HouseholdPandaArm()

    rand_nos = []
    for i in range (6):
        if i<3:
            rand_no = (DinnerTablePanda.getRandomFloat('x'))
            rand_to_float = float("%.2f" % rand_no)
            #print(rand_to_float)
            rand_nos.append(rand_to_float)
        else:
            rand_no = (DinnerTablePanda.getRandomFloat('y'))
            rand_to_float = float("%.2f" % rand_no)
            #print(rand_to_float)
            rand_nos.append(rand_to_float)


    # check distance between objects

    distanced_y = False


    while not distanced_y:
        if (abs(rand_nos[3] - rand_nos[4]) < 0.1):
            rand_nos[3] = float("%.2f" % (DinnerTablePanda.getRandomFloat('y')))
            firstCheck = False
        else:
            firstCheck = True
        if (abs(rand_nos[3] - rand_nos[5]) < 0.1):
            rand_nos[5] = float("%.2f" % (DinnerTablePanda.getRandomFloat('y')))
            secondCheck = False
        else:
            secondCheck = True
        if (abs(rand_nos[4] - rand_nos[5]) < 0.1):
            rand_nos[4] = float("%.2f" % (DinnerTablePanda.getRandomFloat('y')))
            firstCheck = False
            secondCheck = False
        else:
            thirdCheck = True

        if firstCheck and secondCheck and thirdCheck:
            distanced_y = True

    # objects are now in random positions, adequately spaced apart.




    # initiailise object characteristics
    table_name = DinnerTablePanda.table_name
    table_coordinates = (DinnerTablePanda.table_x, DinnerTablePanda.table_y, DinnerTablePanda.table_z)
    table_dimensions = (DinnerTablePanda.table_size_x, DinnerTablePanda.table_size_y, DinnerTablePanda.table_size_z)

    plate_name = DinnerTablePanda.plate_name
    random_plate_coordinates = (rand_nos[0], rand_nos[3], DinnerTablePanda.plate_z)
    plate_dimensions = (DinnerTablePanda.plate_size_x, DinnerTablePanda.plate_size_y, DinnerTablePanda.plate_size_z)

    knife_name = DinnerTablePanda.knife_name
    random_knife_coordinates = (rand_nos[1], rand_nos[4], DinnerTablePanda.knife_z)
    knife_dimensions = (DinnerTablePanda.knife_size_x, DinnerTablePanda.knife_size_y, DinnerTablePanda.knife_size_z)

    fork_name = DinnerTablePanda.fork_name
    random_fork_coordinates = (rand_nos[2], rand_nos[5], DinnerTablePanda.fork_z)
    fork_dimensions = (DinnerTablePanda.fork_size_x, DinnerTablePanda.fork_size_y, DinnerTablePanda.fork_size_z)

    #obj_names = [plate_name, knife_name, fork_name]

    #set the initial random positions of the objets
    DinnerTablePanda.set_coordinates(plate_name, random_plate_coordinates)
    DinnerTablePanda.set_coordinates(knife_name, random_knife_coordinates)
    DinnerTablePanda.set_coordinates(fork_name, random_fork_coordinates)


    plate_inplace = False
    knife_inplace = False
    fork_inplace = False
    objects_sorted = False



    print "============ Adding table ..."
    #raw_input()
    time.sleep(3)
    DinnerTablePanda.add_object(table_name, table_coordinates, table_dimensions)
    time.sleep(1)

    print "============ Adding first object (plate)..."
    #raw_input()
    DinnerTablePanda.add_object(plate_name, random_plate_coordinates, plate_dimensions)
    time.sleep(1)

    print "============ Adding second object (knife)..."
    #raw_input()
    DinnerTablePanda.add_object(knife_name, random_knife_coordinates, knife_dimensions)
    time.sleep(1)

    print "============ Adding third object (fork)..."
    #raw_input()
    DinnerTablePanda.add_object(fork_name, random_fork_coordinates, fork_dimensions)
    time.sleep(1)
    print ""
    print "============ SCENE READY!"
    print ""
    #print "============ calibrate arm to surface to ensure everything is in reach"
    #raw_input()
    #DinnerTablePanda.calibrate_arm()


    #SELECT MODE
    print "============ EASY MODE configures only if knife and fork"
    print "============= are to the left or right of the plate."
    print "============ ADVANCED MODE configures specific coordinates"
    print "============= for the plate, knife and fork."

    questions = [
        inquirer.List('mode',
                message="Select MODE",
                choices=['EASY', 'ADVANCED'],
                carousel=True
            ),
    ]
    answer = inquirer.prompt(questions)

    #BEGIN EASY MODE
    if answer['mode'] == 'EASY':
        print("======================EASY MODE========================")
        rightful_plate_coordinates = (0.4, 0)
        knife_positions = []
        fork_positions = []


        counter = 1
        while(True):
            if counter == 1:
                print("Demo 1:")
            else:
                print("Demo " + str(counter) + ":")
                ask_to_continue = [
                    inquirer.List('continuing',
                            message="CONTINUE?",
                            choices=['YES', 'NO'],
                            carousel=True
                        ),
                ]

                user_answer = inquirer.prompt(ask_to_continue)
                if user_answer['continuing'] == 'NO' :
                    break


            knife_pos = [
                inquirer.List('position',
                        message="Knife on left or right of plate?",
                        choices=['LEFT', 'RIGHT'],
                        carousel=True
                    ),
            ]
            knife_answer = inquirer.prompt(knife_pos)
            knife_positions.append(knife_answer['position'])

            fork_pos = [
                inquirer.List('position',
                        message="Fork on left or right of plate?",
                        choices=['LEFT', 'RIGHT'],
                        carousel=True
                    ),
            ]
            fork_answer = inquirer.prompt(fork_pos)
            fork_positions.append(fork_answer['position'])

            counter += 1

        no_of_demos = counter-1

        print("User Demonstration Complete. Number of Demos Provided: " + str(no_of_demos))
        knife_data = Counter(knife_positions)
        knife_most_common = max(knife_positions, key=knife_data.get)

        fork_data = Counter(fork_positions)
        fork_most_common = max(fork_positions, key=fork_data.get)


        if knife_most_common == fork_most_common:
            if knife_most_common == "RIGHT":
                rightful_fork_coordinates = (0.4, 0.18)
                rightful_knife_coordinates = (0.4, 0.3)
            else:
                rightful_knife_coordinates = (0.4, -0.18)
                rightful_fork_coordinates = (0.4, -0.3)

        else:
            if knife_most_common == "RIGHT":
                rightful_knife_coordinates = (0.4, 0.2)
                rightful_fork_coordinates = (0.4, -0.2)
            else:
                rightful_knife_coordinates = (0.4, -0.2)
                rightful_fork_coordinates = (0.4, 0.2)
    #END EASY MODE

    #BEGIN ADVANCED MODE
    else:
        print("======================ADVANCED MODE========================")
        print "============ Input coordinates in the format 'x, y' ,"
        print "without parentheses and separated by a comma and a space."
        print ""
        print("X bounds: {} , {}".format(DinnerTablePanda.table_lower_x, DinnerTablePanda.table_upper_x))

        print("Y bounds: {} , {}".format(DinnerTablePanda.table_lower_y, DinnerTablePanda.table_upper_y))

        print ""



        good_vals = False
        while not good_vals:
            counter = 1
            given_plate_coordinates = []
            given_knife_coordinates = []
            given_fork_coordinates = []

            while(True):
                if counter == 1:
                    msg = ("Demo " + str(counter) + ": BEGIN?")

                else:
                    msg = ("Demo " + str(counter) + ": CONTINUE?")


                USER_CONFIRM_NEW_DEMO = [
                    inquirer.List('demo',
                        message=(msg),
                        choices=['YES', 'NO'],
                        carousel=True
                    ),
                ]
                USER_IN = inquirer.prompt(USER_CONFIRM_NEW_DEMO)
                USER_ANS = (USER_IN['demo'])

                if counter == 1:
                    if USER_ANS == "NO":
                        sys.exit()

                else:
                    if USER_ANS == "NO":
                        break


                inputs = [">>>PLATE: ", ">>>KNIFE: ", ">>>FORK: "]
                for i in range(3):
                    user_input = raw_input(inputs[i])
                    if ", " in user_input:
                        new_list = user_input.split(", ")
                        if len(new_list) == 2:
                            user_x_as_float = float(new_list[0])
                            user_y_as_float = float(new_list[1])
                            user_x_rounded = "%.2f" % user_x_as_float
                            user_y_rounded = "%.2f" % user_y_as_float
                            if i == 0:
                                given_plate_coordinates.append((user_x_rounded, user_y_rounded))
                            elif i == 1:
                                given_knife_coordinates.append((user_x_rounded, user_y_rounded))
                            else:
                                given_fork_coordinates.append((user_x_rounded, user_y_rounded))


                counter += 1
            #END OF USER DEMO

            #generate best coordinates for objects, given the user's demos
            given_coordinates = [given_plate_coordinates, given_knife_coordinates, given_fork_coordinates]

            for j in range(3):
                obj_list = given_coordinates[j]
                obj_x_total = 0
                obj_y_total = 0

                for pair in obj_list:
                    obj_x_total += float(pair[0])
                    obj_y_total += float(pair[1])
                x_avg = float("%.2f" % (obj_x_total / len(obj_list)))
                y_avg = float("%.2f" % (obj_y_total / len(obj_list)))

                if j == 0:
                    rightful_plate_coordinates = (x_avg, y_avg)
                elif j == 1:
                    rightful_knife_coordinates = (x_avg, y_avg)
                else:
                    rightful_fork_coordinates = (x_avg, y_avg)

            error_msg1 = "ERROR - ENTER DIFFERENT COORDINATES FOR ALL OBJECTS, WITHIN THE BOUNDARIES"
            error_msg2 = "RESTARTING DEMONSTRATION SESSION NOW"

            if rightful_plate_coordinates[0] == rightful_knife_coordinates[0]:
                if rightful_plate_coordinates[1] == rightful_knife_coordinates[1]:
                    print(error_msg1)
                    print ""
                    print(error_msg2)

            elif rightful_plate_coordinates[0] == rightful_fork_coordinates[0]:
                if rightful_plate_coordinates[1] == rightful_fork_coordinates[1]:
                    print(error_msg1)
                    print ""
                    print(error_msg2)
            elif rightful_knife_coordinates[0] == rightful_fork_coordinates[0]:
                if rightful_knife_coordinates[1] == rightful_fork_coordinates[1]:
                    print(error_msg1)
                    print ""
                    print(error_msg2)

            bound_check = True
            fin_coordinates = [rightful_plate_coordinates, rightful_knife_coordinates, rightful_fork_coordinates]
            for q in range(3):
                xy_vals = fin_coordinates[q]

                if not DinnerTablePanda.table_lower_x <= xy_vals[0] <= DinnerTablePanda.table_upper_x:
                    bound_check = False
                    print(error_msg1)
                    print ""
                    print(error_msg2)

                elif not DinnerTablePanda.table_lower_y <= xy_vals[1] <= DinnerTablePanda.table_upper_y:
                    bound_check = False
                    print(error_msg1)
                    print ""
                    print(error_msg2)

            if bound_check:
                good_vals = True
                no_of_demos = counter-1

                print("User Demonstration Complete. Number of Demos Provided: " + str(no_of_demos))



    #END ADVANCED MODE

    #SORT OBJECTS
    obj_ids = [plate_name, knife_name, fork_name]
    obj_coordinates = [rightful_plate_coordinates, rightful_knife_coordinates, rightful_fork_coordinates]
    output_1 = ["plate now", "knife now", "fork now"]
    output_2 = ["plate done", "knife done", "fork done"]

    while not objects_sorted:

        for k in range(3):

            obj_inplace = DinnerTablePanda.check_object(obj_ids[k], obj_coordinates[k], False)
            if not obj_inplace:
                print(output_1[k])
                DinnerTablePanda.attach_object(obj_ids[k])
                DinnerTablePanda.try_move_to_goal(obj_ids[k], obj_coordinates[k], True)
                DinnerTablePanda.detach_object(obj_ids[k])
            else:
                print(output_2[k])
                if k == 0:
                    plate_inplace = obj_inplace
                elif k == 1:
                    knife_inplace = obj_inplace
                else:
                    fork_inplace = obj_inplace

        if (plate_inplace and knife_inplace and fork_inplace):
            objects_sorted = True
            print("========== SORTED! ==========")
            DinnerTablePanda.finish_move()
            sys.exit()







  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
