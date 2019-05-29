;; Auto-generated. Do not edit!


(when (boundp 'swiftpro::SwiftproState)
  (if (not (find-package "SWIFTPRO"))
    (make-package "SWIFTPRO"))
  (shadow 'SwiftproState (find-package "SWIFTPRO")))
(unless (find-package "SWIFTPRO::SWIFTPROSTATE")
  (make-package "SWIFTPRO::SWIFTPROSTATE"))

(in-package "ROS")
;;//! \htmlinclude SwiftproState.msg.html


(defclass swiftpro::SwiftproState
  :super ros::object
  :slots (_motor_angle1 _motor_angle2 _motor_angle3 _motor_angle4 _x _y _z _pump _swiftpro_status _gripper ))

(defmethod swiftpro::SwiftproState
  (:init
   (&key
    ((:motor_angle1 __motor_angle1) 0.0)
    ((:motor_angle2 __motor_angle2) 0.0)
    ((:motor_angle3 __motor_angle3) 0.0)
    ((:motor_angle4 __motor_angle4) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:pump __pump) 0)
    ((:swiftpro_status __swiftpro_status) 0)
    ((:gripper __gripper) 0)
    )
   (send-super :init)
   (setq _motor_angle1 (float __motor_angle1))
   (setq _motor_angle2 (float __motor_angle2))
   (setq _motor_angle3 (float __motor_angle3))
   (setq _motor_angle4 (float __motor_angle4))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _pump (round __pump))
   (setq _swiftpro_status (round __swiftpro_status))
   (setq _gripper (round __gripper))
   self)
  (:motor_angle1
   (&optional __motor_angle1)
   (if __motor_angle1 (setq _motor_angle1 __motor_angle1)) _motor_angle1)
  (:motor_angle2
   (&optional __motor_angle2)
   (if __motor_angle2 (setq _motor_angle2 __motor_angle2)) _motor_angle2)
  (:motor_angle3
   (&optional __motor_angle3)
   (if __motor_angle3 (setq _motor_angle3 __motor_angle3)) _motor_angle3)
  (:motor_angle4
   (&optional __motor_angle4)
   (if __motor_angle4 (setq _motor_angle4 __motor_angle4)) _motor_angle4)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:pump
   (&optional __pump)
   (if __pump (setq _pump __pump)) _pump)
  (:swiftpro_status
   (&optional __swiftpro_status)
   (if __swiftpro_status (setq _swiftpro_status __swiftpro_status)) _swiftpro_status)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:serialization-length
   ()
   (+
    ;; float64 _motor_angle1
    8
    ;; float64 _motor_angle2
    8
    ;; float64 _motor_angle3
    8
    ;; float64 _motor_angle4
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; uint8 _pump
    1
    ;; uint8 _swiftpro_status
    1
    ;; uint8 _gripper
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _motor_angle1
       (sys::poke _motor_angle1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _motor_angle2
       (sys::poke _motor_angle2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _motor_angle3
       (sys::poke _motor_angle3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _motor_angle4
       (sys::poke _motor_angle4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _pump
       (write-byte _pump s)
     ;; uint8 _swiftpro_status
       (write-byte _swiftpro_status s)
     ;; uint8 _gripper
       (write-byte _gripper s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _motor_angle1
     (setq _motor_angle1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _motor_angle2
     (setq _motor_angle2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _motor_angle3
     (setq _motor_angle3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _motor_angle4
     (setq _motor_angle4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _pump
     (setq _pump (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _swiftpro_status
     (setq _swiftpro_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gripper
     (setq _gripper (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get swiftpro::SwiftproState :md5sum-) "bcd9671f860a15ba5765d673098d21bb")
(setf (get swiftpro::SwiftproState :datatype-) "swiftpro/SwiftproState")
(setf (get swiftpro::SwiftproState :definition-)
      "float64 motor_angle1
float64 motor_angle2
float64 motor_angle3
float64 motor_angle4
float64 x
float64 y
float64 z
uint8 	pump
uint8 	swiftpro_status
uint8 	gripper


")



(provide :swiftpro/SwiftproState "bcd9671f860a15ba5765d673098d21bb")


