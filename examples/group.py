#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math
import argparse
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from sympy import Point3D, Line3D, Ray3D, Segment3D, Plane
from enum import Enum, auto
from simple_pid import PID

parser = argparse.ArgumentParser(description='Number of drones')
parser.add_argument('num_drones', metavar='N', type=int,
                    help='number of drones')
args = parser.parse_args()

instances_num = args.num_drones #количество аппаратов
freq = 20 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {}
lz = {}
formation_string = "string"
formation = []
formation_global = []
formation_old = []
keys_old = [i for i in range(instances_num)]
changed_sector = False
out_of_form = False
is_finished = False


ref_point1 = np.array([41, -72, 10], dtype='float64')
ref_point2 = np.array([-41, 72, 10], dtype='float64')
t0 = 0
drone_i = 0
coor_changed = False

class States(Enum):
    ROAM = auto()
    TRANSITION = auto()

class Drone:
  def __init__(self, position, size=5):
    self.position = position #Стартовая позиция
    #self.velocity_mod = 0 #Скорость
    self.size = size #Размер
    self.target_point = [0, 0, 0]
    self.pid_x = PID(1.5, 0, 0, setpoint=self.target_point[0])
    self.pid_y = PID(1.5, 0, 0, setpoint=self.target_point[1])
    self.pid_z = PID(1.5, 0, 0, setpoint=self.target_point[2])
    self.pid_x.output_limits = (-12, 12)
    self.pid_y.output_limits = (-12, 12)
    self.pid_z.output_limits = (-1, 3)
    self.pid_x.sample_time = 0.01
    self.pid_y.sample_time = 0.01
    self.pid_z.sample_time = 0.01
    self.start_time = None
    self.crossed_transition = True
    

  def setTarget(self, target):
    #Функция задания целевой точки
    self.target = target
    print('\n&&&&&&')
    print('tg', self.target)
    print('pos', self.position)
    
    direction = [self.target[0] - self.position[0], self.target[1] - self.position[1], self.target[2] - self.position[2] ]
    norm = np.linalg.norm([float(direction[i]) for i in range(3)])
    self.direction = direction / norm
    print('dir in set', self.direction)
    print('&&&&&&\n')
    #self.velocity = Vector3D(direction)
    #self.velocity_mod = self.velocity.length()

  def updDirection(self):
    direction = [self.target[0] - self.position[0], self.target[1] - self.position[1], self.target[2] - self.position[2] ]
    norm = np.linalg.norm([float(direction[i]) for i in range(3)])
    self.direction = direction / norm


class Drones:
  def __init__(self, data):
    print('initiating')

    pt = PositionTarget()
    #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    self.exist = True
    self.numberOfDrones = instances_num 
    self.positions = []
    self.targets = []
    self.drones = []
    self.are_ready = False
    self.reached_formation_old = [True] * instances_num

    self.cur_state = States.TRANSITION
    self.cur_direction = [1, 0, 0]
    self.speed = 11.5
    self.formation_assumed = [False] * instances_num
    self.formation_reached = [False] * instances_num
    self.ref_point = np.array([0, 0, 0], dtype='float64')
    self.roam_start_time = 0

    try:
      for i in range(self.numberOfDrones):
        self.positions.append(data[i+1]['local_position/pose'].pose.position)
      for i in range(len(self.drones)):
        self.drones[i].position = [self.positions[i].x,self.positions[i].y,self.positions[i].z]
      self.matrix = np.zeros((instances_num, instances_num))
    except Exception:
      print('exception in drones init')
      self.exist = False
    
    if (len(self.positions)==instances_num):
      for i in range(self.numberOfDrones):
        pos = self.positions[i]
        self.drones.append(Drone(Point3D([pos.x, pos.y, pos.z])))
        
  def setCurrentState(self):
    global changed_sector
    turnCounter = 0
    # Potential bug
    # for pos in self.positions:
    #pos = self.positions[0]
    check_arr = [False] * instances_num
    for i in range(instances_num):
      if (self.positions[i].y < -62 or self.positions[i].y > 62) and not self.are_ready:
        check_arr[i] = True
    # print('areready', self.are_ready)
    #print('pos in setstate', pos)
    # print('self pos', self.positions)
    # print('ca', check_arr)
    if all(check_arr) and all(self.reached_formation_old):
      self.reached_formation_old = [False] * instances_num
      if self.cur_state == States.ROAM:
        changed_sector = True
        
      self.cur_state = States.TRANSITION
      for drone in self.drones:
        drone.pid_y.Kd = 0

      print("changing to transition")
    elif not any(check_arr):
      if self.cur_state == States.TRANSITION:
        for drone in self.drones:
          drone.crossed_transition = False
      self.cur_state = States.ROAM
      print("changing to roam")


  def build_order(self):
    global ref_point1
    global ref_point2
    order_def = [i for i in range(instances_num)]
    #print('building order', formation_global)
    if (self.ref_point[1] == ref_point1[1]):
      sort_func = lambda i: -formation_global[i][0]
      #print('rf1')
    elif self.ref_point[1] == ref_point2[1]:
      sort_func = lambda i: formation_global[i][0]
      #print('rf2')
    else:
      #print('wrong ref point')
      return 0
    #print('keys', [formation_global[i][0] for i in range(len(order_def))])
    self.order =  sorted(order_def, key=sort_func) 
    #print('sorted build', self.order) 
  
  def processMatrix(self):
    self.find_optimal()
    for i in range(self.numberOfDrones):
      self.drones[i].setTarget(formation_global[self.targets[i]])
    self.build_order()
     

  def setPositions(self, data):
    self.numberOfDrones = len(data.keys())
    for i in range(self.numberOfDrones):
      self.positions[i] = data[i+1]['local_position/pose'].pose.position
    for i in range(len(self.drones)):
      self.drones[i].position = [self.positions[i].x,self.positions[i].y,self.positions[i].z]
      
    

  def find_optimal(self):
    print('!!!!\n')
    print('find optimal')
    print('!!!!\n')
    global formation_old
    if self.positions[0].y < -62 or (np.abs(self.positions[0].x) < 5 and self.positions[0].y <= 0):
      self.ref_point = ref_point1
      # print('recheck')
    else: 
      # print('ch1 ', self.positions[0].y < -62)
      # print('ch2 ',np.abs(self.positions[0].x) < 5)
      # print('ch3 ',self.positions[0].y < 0)
      self.ref_point = ref_point2
    formation_global = change_coor_system(self.ref_point)
    form_cur = [drones_global.positions[i] for i in range(len(drones_global.positions))]
    form_next = [formation_global[i] for i in range(len(formation_global))]
    print('forn_cur', form_cur)
    if (formation_global[0][0] > 0):
      shift = 20
    else:
      shift = -20

    f1 = lambda p: np.abs(p.x - shift)
    f2 = lambda p: -np.abs(p[0])
    

    # if self.ref_point[1] == ref_point1[1]:
    #   f2 = lambda p: p[0]
    # elif self.ref_point[1] == ref_point2[1]:
    #   print('rf2')
    #   f2 = lambda p: -p[0]
    # else:
    #   print('wrong ref point')
  #     return 0
    form_cur_sorted = sorted(form_cur, key=f1)
    form_next_sorted = sorted(form_next, key=f2)
    # print('forn_cur_sorted', form_cur_sorted)
    # print('forn_next_sorted', form_next_sorted)
    

    drone_numbers = [form_cur.index(form_cur_sorted[i]) for i in range(len(form_cur))]
    # print('dn', drone_numbers)
    #self.order = drone_numbers.copy()
    #self.order.reverse()
    #self.targets = [form_next.index(form_next_sorted[drone_numbers.index(dn)]) for dn in drone_numbers]
    drone_number_indexes = [drone_numbers.index(i) for i in drone_numbers]
    self.targets = [0]*instances_num
    for i in range(len(drone_numbers)):
      drone_number_for_i_pos = drone_numbers[i] #8
      index_of_point = form_next.index(form_next_sorted[i]) #i = 0 index of the most far point
      self.targets[drone_number_for_i_pos] = index_of_point

    #self.targets = [form_next.index(form_next_sorted[dn]) for dn in drone_number_indexes]
    # print('targets', self.targets)

  def find_distance(self, p1, p2):
    #TODO: Добавить учет разницы скоростей по направлениям
    if (type(p2) == type([])):
      p2_p = Point(p2[0], p2[1], p2[2])
    else:
      p2_p = p2
    
    if (type(p1) == type([])):
      p1_p = Point(p1[0], p1[1], p1[2])
    else:
      p1_p = p1
    
    
    res = (p1_p.x - p2_p.x)**2 + (p1_p.y - p2_p.y)**2 + (p1_p.z - p2_p.z)**2
    return res

def change_coor_system(ref_point):
  global formation_global
  global formation
  global coor_changed

  coor_changed = True
  formation_global = []
  turned_formation = [formation[i].copy() for i in range(len(formation))]
  

  # for i in turned_formation:
  #   i[0], i[1] = i[1], i[0]
  for i in range(instances_num):
    formation_global.append([turned_formation[i][j] + ref_point[j]  for j in range(3)])
  return formation_global

def subscribe_on_mavros_topics(suff, data_class):
  #подписываемся на Mavros топики всех аппаратов
  for n in range(1, instances_num + 1):
    data[n] = {}
    topic = f"/mavros{n}/{suff}"
    rospy.Subscriber(topic, data_class, topic_cb, callback_args = (n, suff))

def subscribe_formations(suff, data_class, drones):
  rospy.Subscriber(suff, data_class, formation_cb)

def formation_cb(msg):
  global formation_ar
  global drones_global
  global formation_global
  global formation
  global coor_changed
  global changed_sector
  global is_finished

  msg = str(msg)
  # print(msg)
  name = msg.strip('\"').split(' ')[-1]
  # print(name)
  msg_ar = msg.split(' ')[3:]
  msg_ar2 = []
  if (name == '|'):
    is_finished = True
    print('| is here')
  else:
    for i in msg_ar:
      if not(i == '' or i[-1] == '\\' or i[-1]=='\n'):
        msg_ar2.append(i)
      else:
        check = False
        while (not check):
          if i=='':
            check = True
          elif (i[-1] == '\\' or i[-1]=='\n'):
            i = i[:-1]
          else:
            check = True
        if (i!=''):
          msg_ar2.append(i)
    #msg_ar = []
    msg_ar = msg_ar2.copy()
    if (formation_ar != msg_ar or changed_sector) and drones_global.cur_state == States.TRANSITION :
      print("changing form")
      changed_sector = False
      coor_changed = False
      formation_ar = msg_ar
      formation_string = msg
      formation_temp = formation_ar 
      
    
      formation = []
      for i in range(instances_num):
        formation.append([float(j.strip('\"')) for j in formation_temp[i*3:(i+1)*3]])

      xs = [formation[i][0] for i in range(instances_num)]
      ys = [formation[i][1] for i in range(instances_num)]
      zs = [formation[i][2] for i in range(instances_num)]
      x_c = (min(xs) + max(xs))/2
      y_c = (min(ys) + max(ys))/2
      z_c = (min(zs) + max(zs))/2

      for i in range(instances_num):
        formation[i] = [formation[i][0] - x_c, formation[i][1] - y_c, formation[i][2] - z_c]

      print('form before', formation)
      if len(drones_global.positions) > 0:
        print('pos0', drones_global.positions[0])
      if len(drones_global.positions) > 0 and drones_global.positions[0].x > 0 and drones_global.positions[0].y > 40:
        for i in range(instances_num):
          print('changing')
          formation[i] = [ (-1)*formation[i][0], (-1)*formation[i][1], formation[i][2]]

      # except Exception:
      #   print("exception in formation cb")
      print('form after', formation)
        
      
      print('form', formation)

      coor_changed = False
      formation_global = change_coor_system(drones_global.ref_point)
      if not drones_global.exist:
        print('global does not exist')
        drones_global = Drones(data)
        formation_ar = []
      else:
        
        if drones_global.positions[0].y < -62:
          drones_global.ref_point = ref_point1
        else:
          drones_global.ref_point = ref_point2
      
        coor_changed = False
        formation_global = change_coor_system(drones_global.ref_point)
        

        drones_global.processMatrix()  
      
def topic_cb(msg, callback_args):
  global drones_global
  n, suff = callback_args
  data[n][suff] = msg
  try:
    drones_global.setPositions(data)
  except Exception:
    print("Exception in topic_cb")

def service_proxy(n, path, arg_type, *args, **kwds):
  service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
  ret = service(*args, **kwds)

  #rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")

def arming(n, to_arm):
  d = data[n].get("state")
  if d is not None and d.armed != to_arm:
    service_proxy(n, "cmd/arming", CommandBool, to_arm)

def set_mode(n, new_mode):
  d = data[n].get("state")
  if d is not None and d.mode != new_mode:
    service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)

def subscribe_on_topics():
  # глобальная (GPS) система координат
  subscribe_on_mavros_topics("global_position/global", NavSatFix)

  #локальная система координат, точка отсчета = место включения аппарата
  subscribe_on_mavros_topics("local_position/pose", PoseStamped)
  subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

  #состояние
  subscribe_on_mavros_topics("state", State)
  subscribe_on_mavros_topics("extended_state", ExtendedState)

  #formation
  subscribe_formations("formations_generator/formation", String, drones_global)

def on_shutdown_cb():
  rospy.logfatal("shutdown")

#ROS/Mavros работают в системе координат ENU(Восток-Север-Вверх), автопилот px4 и протокол сообщений Mavlink используют систему координат NED(Север-Восток-Вниз)
#см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

#Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
  pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Смещение на восток
  pt.position.x = x
  #Смещение на север
  pt.position.y = y
  #Высота, направление вверх
  pt.position.z = z

#Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Скорость, направление вверх
  pt.velocity.z = vz

#Управление по vx, vy, pz, локальная система координат.
def set_vxy_pz(pt, vx, vy, pz):
  # Не игнорируем vz, из за особенностей работы автопилота px4.
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #обнуляем, но не используем в управлении
  pt.velocity.z = 0

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Высота, направление вверх
  pt.position.z = pz

#пример управления коптерами
def mc_example(pt, n, dt):

  global drones_global
  global formation_old
  global formation_global
  global keys_old
  try:
    pass
    # print(drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z)
  except Exception:
    pass
  if dt > 0:
    arming(n, True)
  if dt> 0 and dt < 5 and len(drones_global.positions) == instances_num:
    set_vel(pt, 0, 0, 1)
    #set_pos(pt, 0, n*3 - 90, 5)
    formation_old = [[drones_global.positions[i].x, drones_global.positions[i].y, 2] for i in range(instances_num)]
    #print('fo0', formation_old)
    drones_global.ref_point = ref_point1
  # print('rf1', ref_point1)
    formation_global = change_coor_system(ref_point1)
  # print('fg', formation_global)

  if dt>5 and len(drones_global.positions) == instances_num:
    #print(drones_global.cur_state)
    if drones_global.cur_state == States.ROAM:
      # print("roam")
      drones_global.formation_assumed = [False for i in range(instances_num)]
      drones_global.formation_reached = [False] * instances_num
      drones_global.safe = [False] * instances_num
      #drones_global.inline = [False] * instances_num
      
      if drones_global.positions[0].x > 0:
        if drones_global.positions[0].y > 0:
          drones_global.are_ready = False

        drones_global.cur_direction = [0, 1, 0]

        drones_global.ref_point = ref_point1 + np.array(drones_global.cur_direction, dtype='float64') * float(drones_global.speed) * 0.83 * (dt - drones_global.roam_start_time)
        formation_global = change_coor_system(drones_global.ref_point)

        if drones_global.drones[n - 1].position[1] > 58:
          drones_global.drones[n - 1].crossed_transition = True
          drones_global.drones[n-1].target_point = formation_old[keys_old[ n - 1 ]]
          # drones_global.drones[n - 1].pid_y.Kp = 2
          drones_global.drones[n - 1].pid_y.Kd = 0.5
          #set_pos(pt, formation_global[k][0], formation_global[k][1] + 144, formation_global[k][2])

          k = keys_old[n-1]
          dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_old[k])
          if dist < 0.5:
            drones_global.reached_formation_old[n-1] = True
  
        else:
          k = drones_global.targets[n-1]
          #dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_global[k])
          drones_global.drones[n-1].target_point = formation_global[k]
          drones_global.drones[n - 1].pid_y.Kd = 0
        drones_global.drones[n-1].pid_x.setpoint = drones_global.drones[n-1].target_point[0]
        drones_global.drones[n-1].pid_y.setpoint = drones_global.drones[n-1].target_point[1]
        drones_global.drones[n-1].pid_z.setpoint = drones_global.drones[n-1].target_point[2]

        vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
        vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
        vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
        
        set_vel(pt, vel_x, vel_y, vel_z)

      else:
        if drones_global.positions[0].y < 0:
          drones_global.are_ready = False


        drones_global.cur_direction = [0, -1, 0]

        drones_global.ref_point = ref_point2 + np.array(drones_global.cur_direction, dtype='float64') * float(drones_global.speed) * 0.83 * (dt - drones_global.roam_start_time)
        # print('rf', drones_global.ref_point)
        formation_global = change_coor_system(drones_global.ref_point)

        if drones_global.drones[n - 1].position[1] < -58:
          drones_global.drones[n - 1].crossed_transition = True
          drones_global.drones[n-1].target_point = formation_old[keys_old[ n - 1 ]]
          # drones_global.drones[n - 1].pid_y.Kp = 2
          drones_global.drones[n - 1].pid_y.Kd = 0.5
          #set_pos(pt, formation_global[k][0], formation_global[k][1] + 144, formation_global[k][2])

          k = keys_old[n-1]
          dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_old[k])
          if dist < 0.5:
            drones_global.reached_formation_old[n-1] = True
  
        else:
          k = drones_global.targets[n-1]
          #dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_global[k])
          drones_global.drones[n-1].target_point = formation_global[k]
          drones_global.drones[n - 1].pid_y.Kd = 0
        drones_global.drones[n-1].pid_x.setpoint = drones_global.drones[n-1].target_point[0]
        drones_global.drones[n-1].pid_y.setpoint = drones_global.drones[n-1].target_point[1]
        drones_global.drones[n-1].pid_z.setpoint = drones_global.drones[n-1].target_point[2]

        vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
        vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
        vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
        
        set_vel(pt, vel_x, vel_y, vel_z)

    else:
      
      if is_finished:
        k = keys_old[n - 1]
            # print(formation_old)
        set_pos(pt, formation_old[k][0], formation_old[k][1], formation_old[k][2])
      else:
        if drones_global.positions[0].y < -62 or (np.abs(drones_global.positions[0].x) < 5 and drones_global.positions[0].y < 0):
          drones_global.ref_point = ref_point1
        else:
          drones_global.ref_point = ref_point2

        formation_global = change_coor_system(drones_global.ref_point)

        global drone_i
        global t0
        global out_of_form
        k = drones_global.targets[n-1]
        #print('drone_i', drone_i)
        if(drone_i < len(drones_global.order)):
          point_number = drones_global.order[drone_i]
          drone_number = drones_global.targets.index(point_number)
            
          #print('hello 2')
          if (n - 1 == drone_number):
            if drones_global.drones[drone_number].start_time == None:
              drones_global.drones[drone_number].start_time = time.time()

            drones_global.drones[drone_number].updDirection()
            drones_global.drones[n-1].pid_x.Kd = 0.5


            if np.abs(drones_global.positions[drone_number].x) < 10:
              new_target = formation_old[n-1] + drones_global.drones[n-1].direction * drones_global.speed * 0.83 * (time.time() - drones_global.drones[n-1].start_time)
              if (np.abs(new_target[0]) > np.abs(drones_global.drones[n-1].target[0])) and (new_target[0]*drones_global.drones[n-1].target[0] > 0):
                new_target = drones_global.drones[n-1].target
              drones_global.drones[n-1].target_point = new_target
              # print('dn form old', drone_number, formation_old[n-1])
            else:
              k = keys_old[drone_number]
              new_target = formation_old[k] + drones_global.drones[n-1].direction * drones_global.speed * 0.83 * (time.time() - drones_global.drones[n-1].start_time)
              if np.abs(new_target[0]) > np.abs(drones_global.drones[n-1].target[0]) and new_target[0]*drones_global.drones[n-1].target[0] > 0:
                new_target = drones_global.drones[n-1].target

              drones_global.drones[n-1].target_point = new_target
              # print('dn form old', drone_number,  formation_old[k])

            # print('tp', n-1, drones_global.drones[n-1].target_point)

            drones_global.drones[n-1].pid_x.setpoint = drones_global.drones[n-1].target_point[0]
            drones_global.drones[n-1].pid_y.setpoint = drones_global.drones[n-1].target_point[1]
            drones_global.drones[n-1].pid_z.setpoint = drones_global.drones[n-1].target_point[2]

            # print('dn pos', drone_number, drones_global.drones[n-1].position)
            # print('dn form', drone_number, drones_global.drones[n-1].target)
            # print('dn dir', drones_global.drones[n-1].direction)

            
            vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
            vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
            vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
            set_vel(pt, vel_x, vel_y, vel_z)

          elif drones_global.formation_assumed[ n -1 ]:
            
            keep_pos_n = drones_global.targets[n - 1]
            drones_global.drones[n-1].pid_x.setpoint = formation_global[keep_pos_n][0]
            drones_global.drones[n-1].pid_y.setpoint = formation_global[keep_pos_n][1]
            drones_global.drones[n-1].pid_z.setpoint = formation_global[keep_pos_n][2]
            

            vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
            vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
            vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
            set_vel(pt, vel_x, vel_y, vel_z)
            #set_pos(pt, formation_global[keep_pos_n][0], formation_global[keep_pos_n][1], formation_global[keep_pos_n][2])
            # print('fg', keep_pos_n,  formation_global[keep_pos_n])
          else:
            #print("abs", np.abs(drones_global.positions[n-1].x))
            if np.abs(drones_global.positions[n-1].x) < 10:
              set_pos(pt, formation_old[n-1][0], formation_old[n-1][1], formation_old[n-1][2])
              
            else:
              # print("hello")
              k = keys_old[n - 1]
              #drones_global.drones[n - 1].pid_y.output_limits =(-2, 2)
              drones_global.drones[n-1].pid_x.Kd = 0.5
              # print(formation_old)
              drones_global.drones[n-1].pid_x.setpoint = formation_old[k][0]
              drones_global.drones[n-1].pid_y.setpoint = formation_old[k][1]
              drones_global.drones[n-1].pid_z.setpoint = formation_old[k][2]
              

              vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
              vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
              vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
              set_vel(pt, vel_x, vel_y, vel_z)
              #set_pos(pt, formation_old[k][0], formation_old[k][1], formation_old[k][2])
          
          if np.abs(drones_global.positions[drone_number].x) < 10:
            k = keys_old[drone_number]
            #print('fold', formation_old)
            dist_from_old = drones_global.find_distance([drones_global.positions[drone_number].x, drones_global.positions[drone_number ].y, drones_global.positions[drone_number ].z], formation_old[k])
          else:
            k = keys_old[drone_number]
            dist_from_old = drones_global.find_distance([drones_global.positions[drone_number].x, drones_global.positions[drone_number ].y, drones_global.positions[drone_number ].z], formation_old[k])

          # print('dold', drone_number, dist_from_old)

          dist = drones_global.find_distance([drones_global.positions[drone_number].x, drones_global.positions[drone_number ].y, drones_global.positions[drone_number ].z], formation_global[point_number])

          if (dist_from_old > 2 and drone_i < len(drones_global.order) - 1) or dist < 0.5:
            drone_i += 1
            out_of_form = False
            drones_global.formation_assumed[drone_number] = True
            #print('fa', drones_global.formation_assumed)
        else:
          k = drones_global.targets[n-1]
          if all(drones_global.formation_assumed):
            dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1 ].y, drones_global.positions[n-1].z], formation_global[k])
            if dist < 0.5:
              #print(n, 'reached')
              drones_global.formation_reached[n-1] = True
            else:
              set_pos(pt, formation_global[k][0], formation_global[k][1], formation_global[k][2])

            if all(drones_global.formation_reached):

              #print("all in formation")
              for drone in drones_global.drones:
                #print('before none', drone.start_time)
                drone.start_time = None
                drone.pid_x.Kd = 0
              
              drone_i = 0
              if drones_global.positions[0].y < -62:
                drones_global.are_ready = True
                drones_global.cur_state = States.ROAM

                formation_old = formation_global.copy()
                formation_old = [[p[0], p[1] + 144, p[2]] for p in formation_old]
                keys_old = drones_global.targets.copy()
                #print('fotop', formation_old)

                drones_global.roam_start_time = time.time() - t0
              else:
                drones_global.cur_state = States.ROAM

                formation_old = formation_global.copy()
                formation_old = [[p[0], p[1] - 144, p[2]] for p in formation_old]
                keys_old = drones_global.targets.copy()
                #print('fobot', formation_old)

                drones_global.are_ready = True
                drones_global.roam_start_time = time.time() - t0
          else:
            set_pos(pt, formation_global[k][0], formation_global[k][1], formation_global[k][2])

def offboard_loop(mode):
  pub_pt = {}
  #создаем топики, для публикации управляющих значений
  for n in range(1, instances_num + 1):
    pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

  pt = PositionTarget()
  #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  pt.coordinate_frame = pt.FRAME_LOCAL_NED

  global t0
  t0 = time.time()

  #цикл управления
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    
    dt = time.time() - t0
    try:
      global drones_global
      global changed_sector
      drones_global.setCurrentState()
      # print(drones_global.cur_state)
      # print('changed_sec', changed_sector)
    except Exception:
      print("Exception in offboard loop")

    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):

      set_mode(n, "OFFBOARD")

      if mode == 0:
        mc_example(pt, n, dt)
      elif mode == 1:
        vtol_example(pt, n, dt)

      pub_pt[n].publish(pt)
      
    rate.sleep()

if __name__ == '__main__':
  
  drones_global = Drones('')
  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")
  formation_ar = []

  subscribe_on_topics()
  rospy.on_shutdown(on_shutdown_cb)

  m = 0
  

  if len(sys.argv) > 1:
    arg = sys.argv[1]
    if arg == "vtol":
      m = 1

  try:
    
    offboard_loop(m)
  except rospy.ROSInterruptException:
    pass
  rospy.spin()
