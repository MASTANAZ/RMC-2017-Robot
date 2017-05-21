; Auto-generated. Do not edit!


(cl:in-package om17-msg)


;//! \htmlinclude CellCost.msg.html

(cl:defclass <CellCost> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (cost
    :reader cost
    :initarg :cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass CellCost (<CellCost>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CellCost>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CellCost)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name om17-msg:<CellCost> is deprecated: use om17-msg:CellCost instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CellCost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader om17-msg:x-val is deprecated.  Use om17-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CellCost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader om17-msg:y-val is deprecated.  Use om17-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <CellCost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader om17-msg:cost-val is deprecated.  Use om17-msg:cost instead.")
  (cost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CellCost>) ostream)
  "Serializes a message object of type '<CellCost>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CellCost>) istream)
  "Deserializes a message object of type '<CellCost>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cost) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CellCost>)))
  "Returns string type for a message object of type '<CellCost>"
  "om17/CellCost")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CellCost)))
  "Returns string type for a message object of type 'CellCost"
  "om17/CellCost")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CellCost>)))
  "Returns md5sum for a message object of type '<CellCost>"
  "1e3fe3b7498573a946fcc553657ef0ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CellCost)))
  "Returns md5sum for a message object of type 'CellCost"
  "1e3fe3b7498573a946fcc553657ef0ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CellCost>)))
  "Returns full string definition for message of type '<CellCost>"
  (cl:format cl:nil "uint16 x~%uint16 y~%float32 cost~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CellCost)))
  "Returns full string definition for message of type 'CellCost"
  (cl:format cl:nil "uint16 x~%uint16 y~%float32 cost~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CellCost>))
  (cl:+ 0
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CellCost>))
  "Converts a ROS message object to a list"
  (cl:list 'CellCost
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':cost (cost msg))
))
