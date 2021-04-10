#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
import matplotlib.pyplot as plt

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from munkres import Munkres, print_matrix 
from sympy import Point3D, Line3D, Ray3D, Segment3D, Plane
from enum import Enum, auto
from simple_pid import PID

instances_num = 6 #количество аппаратов
freq = 20 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {}
lz = {}
path_x = []
path_y = []
formation_string = "string"
formation = []
formation_global = []
ref_point1 = np.array([41, 0, 0], dtype='float64')
ref_point2 = np.array([-41, 144, 0], dtype='float64')
t0 = 0
drone_i = 0
coor_changed = False

def turnVectorByAlpha2d(alpha, vec):
  rotMatrix = [[math.cos(alpha), -math.sin(alpha), 0],
               [math.sin(alpha),              math.cos(alpha),   0              ],
               [0,0,   1]]

  result = [0, 0, 0]
  for i in range(3):
    for j in range(3):
      result[i] += rotMatrix[i][j] * vec[j]

  return result

class States(Enum):
    ROAM = auto()
    TRANSITION = auto()

class Vector3D:
  def __init__(self, direction):
    self.x_limits = [-12, 12]
    self.y_limits = [-12, 12]
    self.z_limits = [-1, 3]
    max_dir = max(direction)
    direction_normalized = [i/max_dir for i in direction]
    res = direction_normalized.copy()
    limits = []
    if (res[0] >= 0):
      limits.append(self.x_limits[1])
    else:
      limits.append(self.x_limits[0])
    
    if (res[1] >= 0):
      limits.append(self.y_limits[1])
    else:
      limits.append(self.y_limits[0])

    if (res[2] >= 0):
      limits.append(self.y_limits[1])
    else:
      limits.append(self.y_limits[0])

    index_max = direction.index(max_dir)
    lim_max = limits[index_max]

    for i in range(len(direction_normalized)):
      res[i] = lim_max * direction_normalized[i]
    for i in range(len(direction_normalized)):
      if (np.abs(res[i]) > np.abs(limits[i])):
        koef = limits[i]/res[i]
        res = [koef*j for j in res]

    self.x = res[0]
    self.y = res[1]
    self.z = res[2]

  def length(self):
    return (self.x**2 + self.y**2 + self.z**2)**0.5
  def __eq__(self, other):
    return (self.x == other.x and self.y == other.y and self.z == other.z)
  def __neg__(self):
    return Vector3D(-self.x, -self.y, -self.z)
  def __add__(self, other):
    p = [self.x + other.x, self.y + other.y, self.z + other.z]
    return Vector3D(p)

class Drone:
  def __init__(self, position, size=5):
    self.position = position #Стартовая позиция
    self.velocity_mod = 0 #Скорость
    self.size = size #Размер
    self.target_point = [0, 0, 0]
    self.pid_x = PID(0.5, 0.1, 0.1, setpoint=self.target_point[0])
    self.pid_y = PID(0.5, 0.1, 0.1, setpoint=self.target_point[1])
    self.pid_z = PID(0.5, 0.1, 0.1, setpoint=self.target_point[2])
    self.pid_x.output_limits = (-5, 5)
    self.pid_y.output_limits = (-5, 5)
    self.pid_z.output_limits = (-5, 5)

  def setTarget(self, target):
    #Функция задания целевой точки
    self.target = target
    self.line = Line3D(self.position, self.target) #Прямая, на которой лежит траектория
    self.ray = Ray3D(self.position, self.target) #Луч, идущий из стартовой позиции в направлении цели
    self.segment = Segment3D(self.position, self.target) #Отрезок, соединяющий стартовую позицию с целевой
    self.start_delay = 0 #Задержка старта

    #direction = [self.target.x - self.position.x,self.target.y - self.position.y, self.target.z - self.position.z ]
    direction = [self.target[0] - self.position[0], self.target[1] - self.position[1], self.target[2] - self.position[2] ]
    self.velocity = Vector3D(direction)
    self.velocity_mod = self.velocity.length()

  def countDistanceBetweenParallel(self, l1, l2): 
    #Функция, которая расчитывает расстояние между параллельными прямыми
    return l1.distance(l2.random_point())
  def countDistanceBetweenSkew(self, l1, l2):
    #Функция, которая расчитывает расстояние между непараллельными скрещивающимися прямыми
    l0 = l2.parallel_line(l1.p1)
    pl = Plane(l1.p1, l1.p2, l0.p2)
    l = Line3D(pl.projection(l0.p1), pl.projection(l0.p2))
    #print(float(self.countDistanceBetweenParallel(l, l2)))
    return self.countDistanceBetweenParallel(l, l2)

  def checkCollision(self, other, safety_factor=1.5):
    #Функция, которая проверяет, столкнутся ли два дрона и вносит задержки по времени
    #TODO: Нет учета разницы предельных скоростей по направлениям
    #print('cheking collision')
    safe = False
    linesInter = self.line.intersection(other.line) #Ищем пересечение прямых
    #print('intersection: ', type(linesInter))
    if (type(linesInter) == type(self.line)):
      #Случай совпадающих прямых

      total_delay = self.start_delay - other.start_delay
      self_is_first = not self.ray.contains(other.position)
      #Если self летит впереди
      if self_is_first:
        total_delay = -total_delay
      #Вычисляем расстояние между дронами с учетом их размеров
      dist = self.position.distance(other.position) - self.size/2 - other.size/2
      #Считаем разницу во времени, с которой дроны будут проходить через одни и те же точки
      t = dist/np.abs(self.velocity_mod - other.velocity_mod) + total_delay
      # Если разница во времени отрицательная, добавляем задержку дрону, который летит сзади
      if (t >= 0 ):
        safe = True
      elif self_is_first:
        other.start_delay += np.abs(t)
        safe = False
      else:
        safe = False
        self.start_delay += np.abs(t)
      return safe

    elif (linesInter): 
      #Случай пересекающихся прямых
      dist1 = self.position.distance(linesInter)
      dist2 = other.position.distance(linesInter)
      #Рассчитываем время, через которое дроны прибудут в точку пересечения
      t1 = dist1/self.velocity_mod + self.start_delay
      t2 = dist2/other.velocity_mod + other.start_delay

      safe_is_first = (t1 <= t2)
      #Рассчитываем временные "опасные зоны"
      delta_t1 = (self.size/2)/self.velocity_mod*safety_factor
      delta_t2 = (other.size/2)/other.velocity_mod*safety_factor
      danger_bounds1 = (t1 - delta_t1, t1 + delta_t2)
      danger_bounds2 = (t2 - delta_t1, t2 + delta_t1)
      print('crossing', danger_bounds1, danger_bounds2)
      if (danger_bounds2[1] <= danger_bounds1[0]) or (danger_bounds1[1] <= danger_bounds1[0]):
        safe = True
        return safe
      elif (not safe_is_first):
        self.start_delay += (danger_bounds2[1] - danger_bounds1[0])*1.1
        safe = False
        return safe
      else:
        other.start_delay += (danger_bounds1[1] - danger_bounds0[0])*1.1
        safe = False
        return safe

    else: 
      #Случай скрещивающихся прямых

      #Проверяем на параллельность и рассчитывыем расстояние между прямыми
      # print('no intersecions')
      # print(self.line)
      # print(other.line, '\n')
      
      isParallel = self.line.is_parallel(other.line)
      # print(isParallel)

      if isParallel:
        dist = self.countDistanceBetweenParallel(self.line, other.line)
      else:
        dist = self.countDistanceBetweenSkew(self.line, other.line)
      
      #Если оно больше суммы радиусов дронов, то пролет безопасен
      print('dist', float(dist))
      if (dist > self.size /2 + other.size/2):
        print('safe', float(dist), self.size /2 + other.size/2)
        safe = True
        return safe
      else:
        #Случай параллельных прямых
        if isParallel and (self.velocity == -other.velocity):
          #TODO:НЕ УЧТЕНО В ДРУГИХ СЛУЧАЯХ
          t1 = self.target.copy()
          t2 = other.target.copy()
          self.setTarget(t2)
          other.setTarget(t1)
          safe = False
          return safe
        elif not isParallel:
          #Случай скрещенных непараллельных прямых
          #print('danger!!')
          #Рассчитываем точки пересечения с проекциями
          p1 = self.line.projection(other.line)
          p2 = other.line.projection(self.line)

          if (not self.ray.contains(p1)) or (not other.ray.contains(p2)):
            #Если пересечение на продолжении луча, столкновения не будет
            #print('case 1')
            safe = True
            return safe
          else:
            #print('not case 1')
            #Рассчитываем расстояние до точки пересечения с проекцией
            dist1 = p1.distance(self.position)
            dist2 = p2.distance(other.position) 
            #Рассчитываем время, через которое дрон прибудет в точку пересечения
            t1 = dist1/self.velocity_mod + self.start_delay
            t2 = dist2/other.velocity_mod + other.start_delay

            #Рассчитываем временные "опасные зоны"
            delta_t1 = (self.size/2)/self.velocity_mod*safety_factor
            delta_t2 = (other.size/2)/other.velocity_mod*safety_factor
            danger_bounds1 = (t1 - delta_t1, t1 + delta_t2)
            danger_bounds2 = (t2 - delta_t1, t2 + delta_t1)

            if (danger_bounds2[1] <= danger_bounds1[0]) or (danger_bounds1[1] <= danger_bounds1[0]):
              #print('case 2')
              safe = True
              return safe
            else:
              self.start_delay += (danger_bounds2[1] - danger_bounds1[0])*1.1
              #print('case 3')
              safe = False
              return safe

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

    self.omega = 0.03
    self.cur_state = States.TRANSITION
    self.cur_direction = [1, 0, 0]
    self.speed = 12
    self.formation_assumed = [False for i in range(instances_num)]
    self.formation_reached = [False] * instances_num
    self.ref_point = np.array([0, 0, 0], dtype='float64')
    self.roam_start_time = 0

    self.delays = [0]*6


    try:
      for i in range(self.numberOfDrones):
        self.positions.append(data[i+1]['local_position/pose'].pose.position)
      self.matrix = np.zeros((instances_num, instances_num))
    except Exception:
      print('exception in drones init')
      self.exist = False
    
    if (len(self.positions)==instances_num):
      for i in range(self.numberOfDrones):
        pos = self.positions[i]
        #print("before")
        self.drones.append(Drone(Point3D([pos.x, pos.y, pos.z])))
        #print("after")

  def setCurrentState(self):
    turnCounter = 0
    # for pos in self.positions:
    pos = self.positions[0]

    #print(pos.z - 72)
    if (pos.y - 72 < -62 or pos.y - 72 > 62) and not self.are_ready:
      self.cur_state = States.TRANSITION
    else:
      self.cur_state = States.ROAM
      
    # print(self.cur_state)

  
  def processMatrix(self):
    print("process matrix")
    
    self.calculateDistMatrix()
    self.find_optimal()
    for i in range(self.numberOfDrones):
      self.drones[i].setTarget(formation_global[self.targets[i]])
    self.safe = [False]*6
    print('drones amount', self.numberOfDrones)
    # while (not all(self.safe)):
    #   for i in range(self.numberOfDrones):
    #     safe_single_drone = [False]*6
    #     for j in range(i, self.numberOfDrones):
    #       if (i == j):
    #         safe_single_drone[i] = True
    #       else:
    #         safe_single_drone[j] = self.drones[i].checkCollision(self.drones[j])

    #     #print('for drone', i, safe_single_drone)
    #     if (all(safe_single_drone[i+1:])):
    #       self.safe[i] = True
    self.build_order()
    print(formation_global)
    print(self.order)
    #print('order of points')
    # for dr_n in (self.order):
    #   p_n = self.targets.index() 
    
        

  def setPositions(self, data):
    # print('setting')
    self.numberOfDrones = len(data.keys())
    for i in range(self.numberOfDrones):
      self.positions[i] = data[i+1]['local_position/pose'].pose.position
    
  def calculateDistMatrix(self):
    #print('fg', formation_global)
    self.matrix = np.zeros((6, 6))
    for i in range(6):
      for j in range(6):
        #TODO:enough to calculate upper half 
        self.matrix[i][j] = self.find_distance(self.positions[i], formation_global[j])

  def build_order(self):
    global ref_point1
    global ref_point2
    order_def = [i for i in range(instances_num)]
    print('building order', formation_global)
    if self.ref_point[1] == ref_point1[1]:
      sort_func = lambda i: -formation_global[i][0]
    elif self.ref_point[1] == ref_point2[1]:
      sort_func = lambda i: formation_global[i][0]
    else:
      print('wrong ref point')
      return 0
    #print('keys', [formation_global[i][0] for i in range(len(order_def))])
    self.order =  sorted(order_def, key=sort_func)  
  
  def send_drones(self, pt, n):
    i = 0
    while(i < self.order):
      drone_number = self.targets.index(i)
      if (n == drone_number):
        set_pos(pt, formation_global[i][0], formation_global[i][1], formation_global[i][2])
      
      dist = self.find_distance(self.positions[n - 1])
    

  def find_optimal(self):
    m = Munkres()
    indexes = m.compute(self.matrix.copy())
    
    self.targets = [x[1] for x in indexes]

    #print(self.targets)
  
    
  def find_distance(self, p1, p2):
    #TODO: Добавить учет разницы скоростей по направлениям
    #print('find dist')
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

  
#drones = Drones('')
def change_coor_system(ref_point):
  global formation_global
  global formation
  global coor_changed
  
  #print('f2', formation)
  if (not coor_changed):
    coor_changed = True
    formation_global = []
    turned_formation = [formation[i].copy() for i in range(len(formation))]
    print('fb', formation)
    print('f1w', turned_formation)
    for i in turned_formation:
      i[0], i[1] = i[1], i[0]
    print('f2w', turned_formation)
    print('ff', formation)

    for i in range(instances_num):
      formation_global.append([turned_formation[i][j] + ref_point[j]  for j in range(3)])
    # print(formation_global)
  # print('change coor', formation_global)
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
  #formation = []
  msg = str(msg)
  #print(msg)
  msg_ar = msg.split(' ')[3:]
  if formation_ar != msg_ar:
    # print('fs ', formation_ar)
    # print('msg', msg_ar)
    # print('ref', drones_global.ref_point)
    coor_changed = False
    formation_ar = msg_ar
    formation_string = msg
    formation_temp = formation_ar #formation_string.split(' ')[3:]
    
    #print(formation_string)
    formation = []
    for i in range(6):
      formation.append([float(j.strip('\"')) for j in formation_temp[i*3:(i+1)*3]])
    #print('ref', drones_global.ref_point)
    # print('formation', formation)
    # print('point1')
    coor_changed = False
    formation_global = change_coor_system(drones_global.ref_point)#72, 25])
    # print('fg', formation_global)
    if not drones_global.exist:
      print('global does not exist')
      drones_global = Drones(data)
      formation_ar = []
    else:
      
      if drones_global.positions[0].y - 72 < -62:
        drones_global.ref_point = [41, 0, 0]
      else:
        drones_global.ref_point = [-41, 144, 0]
      # print('point1')
      coor_changed = False
      formation_global = change_coor_system(drones_global.ref_point)#72, 25])
      # formation_global = change_coor_system(drones_global.ref_point)#72, 25])
      drones_global.processMatrix()  
    
def topic_cb(msg, callback_args):
  global drones_global
  n, suff = callback_args
  data[n][suff] = msg
  try:
    drones_global.setPositions(data)
  except Exception:
    pass
    # print("exception")

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

def vtol_to_fw(n):
  d = data[n].get("extended_state")
  if d is not None:
    if d.vtol_state != ExtendedState.VTOL_STATE_FW and d.vtol_state != ExtendedState.VTOL_STATE_TRANSITION_TO_FW:
      service_proxy(n, "cmd/vtol_transition", CommandVtolTransition, state = ExtendedState.VTOL_STATE_FW)

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

#взлет коптера
def mc_takeoff(pt, n, dt):
  if dt<10:
    #скорость вверх
    set_vel(pt, 0, 0, 4)

    #армимся и взлетаем с заданной скоростью
  if dt>5:
    arming(n, True)

#пример управления коптерами
def mc_example(pt, n, dt, tb):
  t_new = time.time()
  if tb is None:
    delta = 0
  else:
    delta =  t_new - tb

  #mc_takeoff(pt, n, dt)
  global drones_global
  global formation_global
  if dt>5:
    arming(n, True)

  if dt>10 and len(drones_global.positions) == instances_num:
    #print(drones_global.cur_state)
    if drones_global.cur_state == States.ROAM:
      print("roam")
      drones_global.formation_assumed = [False for i in range(instances_num)]
      drones_global.formation_reached = [False] * instances_num
      drones_global.safe = [False] * instances_num
      
      if drones_global.positions[0].x > 0:
        if drones_global.positions[0].y > 70:
          drones_global.are_ready = False


        drones_global.cur_direction = [0, 1, 0]

        k = drones_global.targets[n-1]
        # # print(drones_global.roam_start_time)
        # drones_global.ref_point = ref_point1 + np.array(drones_global.cur_direction, dtype='float64') * float(drones_global.speed) * 0.83 * (dt - drones_global.roam_start_time)
        # formation_global = change_coor_system(drones_global.ref_point)

        # dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_global[k])
        # drones_global.drones[n-1].target_point = formation_global[k]
        # drones_global.drones[n-1].pid_x.setpoint = drones_global.drones[n-1].target_point[0]
        # drones_global.drones[n-1].pid_y.setpoint = drones_global.drones[n-1].target_point[1]
        # drones_global.drones[n-1].pid_z.setpoint = drones_global.drones[n-1].target_point[2]

        # # print(n, ' ', formation_global[k])
        # # print(n, ' ', drones_global.positions[n-1])
        # vel_x = drones_global.drones[n-1].pid_x(drones_global.positions[n-1].x)
        # vel_y = drones_global.drones[n-1].pid_y(drones_global.positions[n-1].y)
        # vel_z = drones_global.drones[n-1].pid_z(drones_global.positions[n-1].z)
        # # print(n, ' ', vel_x, vel_y, vel_z)

        # set_vel(pt, drones_global.speed * drones_global.cur_direction[0] + vel_x, drones_global.speed * drones_global.cur_direction[1] + vel_y, drones_global.speed * drones_global.cur_direction[2] + vel_z)
        set_pos(pt, formation_global[k][0], formation_global[k][1] + 144, formation_global[k][2])
        print('target_pt', formation_global[k][0], formation_global[k][1] + 144, formation_global[k][2])



      else:
        if drones_global.positions[0].y < 70:
          drones_global.are_ready = False

        drones_global.cur_direction = [0, -1, 0]
        k = drones_global.targets[n-1]

        # set_vel(pt, drones_global.speed * drones_global.cur_direction[0], drones_global.speed * drones_global.cur_direction[1], drones_global.speed * drones_global.cur_direction[2])
        #print('setting pos', formation_global)
        #print('delays', drones_global.delays)
        set_pos(pt, formation_global[k][0], formation_global[k][1] - 144, formation_global[k][2])
    elif drones_global.cur_state == States.TRANSITION:
      print("transition")
      if drones_global.positions[0].y - 72 < -62:
        drones_global.ref_point = ref_point1
      else:
        drones_global.ref_point = ref_point2
      # print('point1')
      formation_global = change_coor_system(drones_global.ref_point)

      global drone_i
      global t0
      k = drones_global.targets[n-1]
      if(drone_i < len(drones_global.order)):
        #drone_number = drones_global.targets.index(drone_i)
        point_number = drones_global.order[drone_i]
        #print('targets', drones_global.targets)
        #print('di', drone_i)
        drone_number = drones_global.targets.index(point_number)
        #print('n ', n)
        #print('dn ', drone_number)
        if (n - 1 == drone_number): 
          # print('branch1')
          set_pos(pt, formation_global[point_number][0], formation_global[point_number][1], formation_global[point_number][2])
          #print('pn', point_number)
          #print('point', formation_global[point_number])
        elif drones_global.formation_assumed[n -1 ]:
          # print('branch2')
          keep_pos_n = drones_global.targets[n - 1]
          set_pos(pt, formation_global[keep_pos_n][0], formation_global[keep_pos_n][1], formation_global[keep_pos_n][2])
          #print('kn', keep_n)
          #print(' keep point', formation_global[keep_n])
        else:
          # print('branch4')
          # print(drones_global.formation_assumed)
          set_pos(pt, drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z)
        
        dist = drones_global.find_distance([drones_global.positions[drone_number].x, drones_global.positions[drone_number ].y, drones_global.positions[drone_number ].z], formation_global[point_number])
        #print(dist)
        if (dist < 2000 and drone_i < len(drones_global.order) - 1) or dist<0.5:
          drone_i += 1
          # print('dn', drone_number)
          # print('dist', dist)
          drones_global.formation_assumed[drone_number] = True
          print('fa', drones_global.formation_assumed)
          #print(drone_i)
      else:
        k = drones_global.targets[n-1]
        if all(drones_global.formation_assumed):
          dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1 ].y, drones_global.positions[n-1].z], formation_global[k])
          if dist < 0.5:
            print(n, 'reached')
            drones_global.formation_reached[n-1] = True
          else:
            set_pos(pt, formation_global[k][0], formation_global[k][1], formation_global[k][2])

          if all(drones_global.formation_reached):
            print("all in formation")
            drone_i = 0
            if drones_global.positions[0].y - 72 < -62:
              # set_vel(pt, 0, 12, 0)
              drones_global.are_ready = True
              drones_global.cur_state = States.ROAM
              drones_global.roam_start_time = time.time() - t0
            else:
              drones_global.cur_state = States.ROAM
              drones_global.are_ready = True
              # set_vel(pt, 0, -12, 0)
              drones_global.roam_start_time = time.time() - t0
        else:
          set_pos(pt, formation_global[k][0], formation_global[k][1], formation_global[k][2])
          
      
      # k = drones_global.targets[n-1]
      # print(n, ' ', formation_global[k])
      # print(n, ' ', drones_global.positions[n-1])

      # dist = 0
      # dist = drones_global.find_distance([drones_global.positions[n-1].x, drones_global.positions[n-1].y, drones_global.positions[n-1].z], formation_global[k])
      # print(dist)

      # print(drones_global.formation_assumed)
      # if dist > 0.5 and False in drones_global.formation_assumed:
      #   print('setting pos', formation_global)
      #   print('delays', drones_global.delays)
      #   set_pos(pt, formation_global[k][0], formation_global[k][1],formation_global[k][2])
      #   #print('form_gl', formation_global)

      # elif dist < 0.5 and False in drones_global.formation_assumed:
      #   print('setting pos', formation_global)
      #   print('delays', drones_global.delays)
      #   set_pos(pt, formation_global[k][0], formation_global[k][1],formation_global[k][2])
      #   drones_global.formation_assumed[n-1] = True
      # else:
      #   # print('dist < 0.5')
      #   # print(drones_global.formation_assumed)
      #   drones_global.safe = [False] * instances_num
      #   drones_global.are_ready = True
      #   if drones_global.positions[0].y - 72 < -62:
      #     # set_vel(pt, 0, 12, 0)
      #     drones_global.cur_state = States.ROAM
      #     drones_global.roam_start_time = time.time() - t0
      #   else:
      #     drones_global.cur_state = States.ROAM

      #     # set_vel(pt, 0, -12, 0)
      #     drones_global.roam_start_time = time.time() - t0
          

  return t_new

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
    tb = None
    try:
      global drones_global
      drones_global.setCurrentState()
    except Exception:
      pass
      #print(drones_global.cur_state)

    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):

      set_mode(n, "OFFBOARD")

    

      if mode == 0:
        tb = mc_example(pt, n, dt, tb)
      elif mode == 1:
        vtol_example(pt, n, dt)

      pub_pt[n].publish(pt)
      
    rate.sleep()

if __name__ == '__main__':
  
  drones_global = Drones('')
  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")
  formation_ar =[]

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
