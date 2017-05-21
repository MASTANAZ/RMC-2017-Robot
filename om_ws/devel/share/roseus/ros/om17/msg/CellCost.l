;; Auto-generated. Do not edit!


(when (boundp 'om17::CellCost)
  (if (not (find-package "OM17"))
    (make-package "OM17"))
  (shadow 'CellCost (find-package "OM17")))
(unless (find-package "OM17::CELLCOST")
  (make-package "OM17::CELLCOST"))

(in-package "ROS")
;;//! \htmlinclude CellCost.msg.html


(defclass om17::CellCost
  :super ros::object
  :slots (_x _y _cost ))

(defmethod om17::CellCost
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:cost __cost) 0.0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _cost (float __cost))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:serialization-length
   ()
   (+
    ;; uint16 _x
    2
    ;; uint16 _y
    2
    ;; float32 _cost
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _x
       (write-word _x s)
     ;; uint16 _y
       (write-word _y s)
     ;; float32 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _cost
     (setq _cost (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get om17::CellCost :md5sum-) "1e3fe3b7498573a946fcc553657ef0ef")
(setf (get om17::CellCost :datatype-) "om17/CellCost")
(setf (get om17::CellCost :definition-)
      "uint16 x
uint16 y
float32 cost

")



(provide :om17/CellCost "1e3fe3b7498573a946fcc553657ef0ef")


